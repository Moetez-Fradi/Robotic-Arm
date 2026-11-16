#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

// ============================================================================
// RoboticArmPCB Rev E (STM32F103C8T6) Pinout
// ============================================================================
#define PIN_JP1         PB12  // Node ID bit 0 (Active Low to GND)
#define PIN_JP2         PB13  // Node ID bit 1 (Active Low to GND)
#define PIN_JP3         PB14  // Node ID bit 2 (Active Low to GND)
#define PIN_LED         PC13  // On-board Status LED (Active Low)

#define PIN_STEP        PB8   // TMC2209 STEP
#define PIN_DIR         PB9   // TMC2209 DIR
#define PIN_DRV_EN      PB5   // TMC2209 Enable Request (gated by hardware ESTOP_24V)
#define PIN_HOME_SW     PB0   // KW4 Limit Switch (Active Low with internal pull-up)
#define PIN_ESTOP_ADC   PA1   // 100k / 8.2k divider from ESTOP_24V (1.82V nominal)

#define PIN_ENC_SCL     PB10  // I2C2 SCL -> AS5600
#define PIN_ENC_SDA     PB11  // I2C2 SDA -> AS5600

#define PIN_SERVO_1     PB6   // Wrist Axis 5 (20 kg·cm) - Active on Node 4 (ID 3)
#define PIN_SERVO_2     PB7   // Wrist Axis 6 (20 kg·cm) - Active on Node 4 (ID 3)
#define PIN_SERVO_3     PA8   // Gripper Servo           - Active on Node 4 (ID 3)

// ============================================================================
// Per-Axis Mechanical Calibration (Adjust after bench jog test)
// ============================================================================
// Microsteps = 16 (200 * 16 = 3200 steps/rev -> 8.8889 steps/deg at 1:1)
// Axis 2 (Node 1) includes 5.18:1 planetary gearbox -> 46.0444 steps/deg
const float STEPS_PER_DEG[4] = {8.8889f, 46.0444f, 8.8889f, 8.8889f};
const bool  INVERT_DIR[4]    = {false,   false,    false,   false};

// ============================================================================
// Global State
// ============================================================================
TwoWire Wire2(PIN_ENC_SDA, PIN_ENC_SCL);
Servo wrist1, wrist2, gripper;

uint8_t node_id = 0;
volatile int32_t current_steps = 0;
volatile int32_t target_steps = 0;
uint32_t step_interval_us = 800;
uint32_t last_step_us = 0;
uint32_t last_telem_ms = 0;

bool driver_enabled = false;
bool is_homing = false;
bool encoder_ok = false;

int32_t encoder_zero_raw = 0;
int32_t encoder_turns = 0;
uint16_t last_raw_angle = 0;

// ============================================================================
// AS5600 12-Bit Magnetic Encoder (I2C2 @ 0x36)
// ============================================================================
uint16_t readAS5600Raw() {
  Wire2.beginTransmission(0x36);
  Wire2.write(0x0C);  // RAW ANGLE high byte register
  if (Wire2.endTransmission(false) != 0) {
    encoder_ok = false;
    return last_raw_angle;
  }
  Wire2.requestFrom(0x36, 2);
  if (Wire2.available() >= 2) {
    encoder_ok = true;
    uint16_t hi = Wire2.read() & 0x0F;
    uint16_t lo = Wire2.read();
    return (hi << 8) | lo;
  }
  encoder_ok = false;
  return last_raw_angle;
}

int32_t getEncoderCentideg() {
  uint16_t raw = readAS5600Raw();
  int16_t diff = (int16_t)raw - (int16_t)last_raw_angle;
  if (diff > 2048) encoder_turns--;
  else if (diff < -2048) encoder_turns++;
  last_raw_angle = raw;

  int32_t total_ticks = (encoder_turns * 4096) + (int32_t)raw - encoder_zero_raw;
  return (int32_t)((total_ticks * 36000LL) / 4096LL);
}

// ============================================================================
// Bare-Metal STM32F103 bxCAN Peripheral (500 kbps @ 36 MHz APB1)
// ============================================================================
void canInit500k() {
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
  RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

  // PA11 (CAN_RX): Input pull-up | PA12 (CAN_TX): Alternate Function Push-Pull 50 MHz
  GPIOA->CRH &= ~(0xFF << 12);
  GPIOA->CRH |= (0xB << 16) | (0x8 << 12);
  GPIOA->ODR |= (1 << 11);

  CAN1->MCR |= CAN_MCR_INRQ;
  while (!(CAN1->MSR & CAN_MSR_INAK));
  CAN1->MCR &= ~CAN_MCR_SLEEP;
  CAN1->MCR |= CAN_MCR_ABOM;  // Automatic bus-off recovery

  // Bit timing: 36 MHz / (Prescaler=4 * (1 + TS1=15 + TS2=2)) = 500 kbps
  CAN1->BTR = (0 << 24) | (1 << 20) | (14 << 16) | (3 << 0);

  // Filter bank 0: accept all standard 11-bit frames into FIFO 0
  CAN1->FMR |= CAN_FMR_FINIT;
  CAN1->FA1R |= 1;
  CAN1->FS1R |= 1;
  CAN1->sFilterRegister[0].FR1 = 0;
  CAN1->sFilterRegister[0].FR2 = 0;
  CAN1->FMR &= ~CAN_FMR_FINIT;

  CAN1->MCR &= ~CAN_MCR_INRQ;
  while (CAN1->MSR & CAN_MSR_INAK);
}

bool canTransmit(uint16_t std_id, const uint8_t *data, uint8_t dlc) {
  if (!(CAN1->TSR & CAN_TSR_TME0)) return false;
  CAN1->sTxMailBox[0].TIR = (uint32_t)std_id << 21;
  CAN1->sTxMailBox[0].TDTR = dlc & 0x0F;
  CAN1->sTxMailBox[0].TDLR = ((uint32_t)data[3] << 24) | ((uint32_t)data[2] << 16) |
                             ((uint32_t)data[1] << 8)  | data[0];
  CAN1->sTxMailBox[0].TDHR = ((uint32_t)data[7] << 24) | ((uint32_t)data[6] << 16) |
                             ((uint32_t)data[5] << 8)  | data[4];
  CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;
  return true;
}

void processCanRx() {
  while (CAN1->RF0R & CAN_RF0R_FMP0) {
    uint16_t id = (CAN1->sFIFOMailBox[0].RIR >> 21) & 0x7FF;
    uint32_t rdlr = CAN1->sFIFOMailBox[0].RDLR;
    uint32_t rdhr = CAN1->sFIFOMailBox[0].RDHR;
    uint8_t d[8] = {
      (uint8_t)(rdlr), (uint8_t)(rdlr >> 8), (uint8_t)(rdlr >> 16), (uint8_t)(rdlr >> 24),
      (uint8_t)(rdhr), (uint8_t)(rdhr >> 8), (uint8_t)(rdhr >> 16), (uint8_t)(rdhr >> 24)
    };
    CAN1->RF0R |= CAN_RF0R_RFOM0;

    // 1. Target Position Command (0x100 + node_id)
    if (id == (0x100 + node_id)) {
      int32_t target_cdeg;
      uint16_t speed_dps;
      memcpy(&target_cdeg, &d[0], 4);
      memcpy(&speed_dps, &d[4], 2);

      is_homing = false;
      target_steps = (int32_t)((target_cdeg / 100.0f) * STEPS_PER_DEG[node_id]);
      if (speed_dps > 0) {
        float steps_per_sec = speed_dps * STEPS_PER_DEG[node_id];
        step_interval_us = constrain((uint32_t)(1000000.0f / steps_per_sec), 150, 25000);
      }
    }
    // 2. System / Homing Command (0x110 + node_id)
    else if (id == (0x110 + node_id)) {
      uint8_t cmd = d[0];
      if (cmd == 1) {
        is_homing = true;
        driver_enabled = true;
        step_interval_us = (uint32_t)(1000000.0f / (30.0f * STEPS_PER_DEG[node_id]));
        digitalWrite(PIN_DRV_EN, HIGH);
      } else if (cmd == 2) {
        driver_enabled = true;
        digitalWrite(PIN_DRV_EN, HIGH);
      } else if (cmd == 3) {
        driver_enabled = false;
        is_homing = false;
        digitalWrite(PIN_DRV_EN, LOW);
      } else if (cmd == 4) {
        encoder_zero_raw = readAS5600Raw();
        encoder_turns = 0;
        current_steps = 0;
        target_steps = 0;
      }
    }
    // 3. Wrist & Gripper PWM Command (0x120 -> Node 4 only)
    else if (id == 0x120 && node_id == 3) {
      uint16_t s1, s2, grip;
      memcpy(&s1, &d[0], 2);
      memcpy(&s2, &d[2], 2);
      memcpy(&grip, &d[4], 2);
      if (s1 >= 500 && s1 <= 2500) wrist1.writeMicroseconds(s1);
      if (s2 >= 500 && s2 <= 2500) wrist2.writeMicroseconds(s2);
      if (grip >= 500 && grip <= 2500) gripper.writeMicroseconds(grip);
    }
  }
}

// ============================================================================
// Setup & Main Control Loop
// ============================================================================
void setup() {
  analogReadResolution(12);  // Use full 12-bit (0-4095) ADC for ESTOP_SENSE

  pinMode(PIN_JP1, INPUT_PULLUP);
  pinMode(PIN_JP2, INPUT_PULLUP);
  pinMode(PIN_JP3, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

  // Read binary jumper ID (shorted jumper to GND = 1)
  node_id = ((!digitalRead(PIN_JP3)) << 2) |
            ((!digitalRead(PIN_JP2)) << 1) |
            (!digitalRead(PIN_JP1));
  if (node_id > 3) node_id = 0;

  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_DRV_EN, OUTPUT);
  digitalWrite(PIN_DRV_EN, LOW);
  pinMode(PIN_HOME_SW, INPUT_PULLUP);

  Wire2.begin();
  Wire2.setClock(400000);
  last_raw_angle = readAS5600Raw();
  encoder_zero_raw = last_raw_angle;

  // Attach 50 Hz servo outputs on Node 4 (Axis 4)
  if (node_id == 3) {
    wrist1.attach(PIN_SERVO_1);
    wrist2.attach(PIN_SERVO_2);
    gripper.attach(PIN_SERVO_3);
    wrist1.writeMicroseconds(1500);
    wrist2.writeMicroseconds(1500);
    gripper.writeMicroseconds(1150);
  }

  canInit500k();
}

void loop() {
  processCanRx();

  // Monitor ESTOP_24V divider (1.82V nominal -> ~2258 counts; threshold = 1000)
  bool estop_ok = analogRead(PIN_ESTOP_ADC) > 1000;
  if (!estop_ok) {
    driver_enabled = false;
    is_homing = false;
    digitalWrite(PIN_DRV_EN, LOW);
  }

  // Non-blocking Stepper Pulse Generator
  uint32_t now_us = micros();
  if (driver_enabled && estop_ok && (now_us - last_step_us >= step_interval_us)) {
    if (is_homing) {
      if (digitalRead(PIN_HOME_SW) == LOW) {
        is_homing = false;
        current_steps = 0;
        target_steps = 0;
        encoder_zero_raw = readAS5600Raw();
        encoder_turns = 0;
      } else {
        bool dir_pin = INVERT_DIR[node_id] ? HIGH : LOW;
        digitalWrite(PIN_DIR, dir_pin);
        digitalWrite(PIN_STEP, HIGH);
        delayMicroseconds(3);
        digitalWrite(PIN_STEP, LOW);
        last_step_us = now_us;
      }
    } else if (current_steps != target_steps) {
      bool forward = (target_steps > current_steps);
      bool dir_pin = (forward ^ INVERT_DIR[node_id]) ? HIGH : LOW;
      digitalWrite(PIN_DIR, dir_pin);
      digitalWrite(PIN_STEP, HIGH);
      delayMicroseconds(3);
      digitalWrite(PIN_STEP, LOW);
      current_steps += forward ? 1 : -1;
      last_step_us = now_us;
    }
  }

  // 50 Hz Telemetry Broadcast (0x200 + node_id)
  uint32_t now_ms = millis();
  if (now_ms - last_telem_ms >= 20) {
    last_telem_ms = now_ms;
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));

    int32_t actual_cdeg = getEncoderCentideg();
    uint16_t err_flags = encoder_ok ? 0x0000 : 0x0001;

    uint8_t tx[8] = {0};
    memcpy(&tx[0], &actual_cdeg, 4);
    tx[4] = (is_homing ? 0x02 : 0x00) | (driver_enabled ? 0x01 : 0x00);
    tx[5] = estop_ok ? 1 : 0;
    memcpy(&tx[6], &err_flags, 2);
    canTransmit(0x200 + node_id, tx, 8);
  }
}