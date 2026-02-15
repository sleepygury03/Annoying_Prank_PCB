// ATtiny814 (megaTinyCore 2.6.12)
// NO SUPERVISOR + explicit peripheral-off + low-power button wake + RTC interval beeps
//
// Pins used:
//  PB0/PB1 = piezo bridge drive
//  PA4     = button (internal pull-up)
//  PA7     = LED pin NOT USED (forced low)
//  PA3     = NOT USED (clamped OUTPUT LOW so pad/header can't float)
//
// Important fix for "minutes-long ramp after beep":
// - Do NOT leave the piezo pins Hi-Z after a beep.
// - Clamp both PB0 and PB1 LOW and disable their input buffers.
//   This discharges the piezo and prevents it from floating / picking up EMI.

#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

// -------- Pins --------
const uint8_t PIEZO_A = PIN_PB0;
const uint8_t PIEZO_B = PIN_PB1;
const uint8_t BUTTON  = PIN_PA4;   // internal pull-up
const uint8_t LEDPIN  = PIN_PA7;   // NOT USED (forced low)
const uint8_t PA3_PAD = PIN_PA3;   // NOT USED (clamped low)

// -------- Beep settings --------
const uint16_t BEEP_MS_BUTTON   = 200;
const uint16_t BEEP_MS_INTERVAL = 120;
const uint16_t HALF_PERIOD_US   = 125; // ~4 kHz target (approx)

// -------- Flags --------
volatile bool intervalWake = false;
volatile bool buttonWake   = false;

// -------- LFSR --------
uint8_t lfsr = 0xA5;

// -------- crude delays (no Arduino timers) --------
static inline void wait_us(uint16_t us) {
  while (us--) __asm__ __volatile__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
}
static inline void wait_ms(uint16_t ms) {
  while (ms--) {
    for (uint16_t i = 0; i < 250; i++)
      __asm__ __volatile__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
  }
}

// -------- Force LED pin low --------
static inline void ledKillForever() {
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);
  PORTA.PIN7CTRL = (PORTA.PIN7CTRL & ~PORT_ISC_gm) | PORT_ISC_INPUT_DISABLE_gc;
  PORTA.PIN7CTRL &= ~PORT_PULLUPEN_bm;
}

// -------- Piezo clamp (discharge + EMI immunity) --------
static inline void piezoClampLow() {
  // Drive both ends low -> discharges piezo and prevents floating
  pinMode(PIEZO_A, OUTPUT);
  pinMode(PIEZO_B, OUTPUT);
  digitalWrite(PIEZO_A, LOW);
  digitalWrite(PIEZO_B, LOW);

  // Disable input buffers on PB0/PB1 (prevents EMI-induced input-buffer current)
  PORTB.PIN0CTRL = (PORTB.PIN0CTRL & ~PORT_ISC_gm) | PORT_ISC_INPUT_DISABLE_gc;
  PORTB.PIN1CTRL = (PORTB.PIN1CTRL & ~PORT_ISC_gm) | PORT_ISC_INPUT_DISABLE_gc;
}

// Optional: re-enable input buffer sense (not strictly required for output use, but tidy)
static inline void piezoEnableBuffers() {
  PORTB.PIN0CTRL = (PORTB.PIN0CTRL & ~PORT_ISC_gm) | PORT_ISC_INTDISABLE_gc;
  PORTB.PIN1CTRL = (PORTB.PIN1CTRL & ~PORT_ISC_gm) | PORT_ISC_INTDISABLE_gc;
}

// -------- Beep --------
static void beep_ms(uint16_t ms) {
  // Ensure pins are normal before toggling
  piezoEnableBuffers();

  pinMode(PIEZO_A, OUTPUT);
  pinMode(PIEZO_B, OUTPUT);

  uint32_t halfCycles = ((uint32_t)ms * 1000UL) / (uint32_t)HALF_PERIOD_US;
  for (uint32_t i = 0; i < halfCycles; i++) {
    if (i & 1) { VPORTB.OUT &= ~PIN0_bm; VPORTB.OUT |=  PIN1_bm; }
    else       { VPORTB.OUT |=  PIN0_bm; VPORTB.OUT &= ~PIN1_bm; }
    wait_us(HALF_PERIOD_US);
  }

  // IMPORTANT: clamp low after beep (no Hi-Z)
  piezoClampLow();
}

// -------- RNG interval 5–30 min --------
static uint8_t lfsrNext(uint8_t x) {
  uint8_t lsb = x & 1;
  x >>= 1;
  if (lsb) x ^= 0xB8;
  return x;
}
static uint16_t nextIntervalSeconds_5to30min() {
  lfsr = lfsrNext(lfsr);
  uint16_t r = (uint16_t)lfsr << 8;
  lfsr = lfsrNext(lfsr);
  r ^= (uint16_t)lfsr;
  lfsr = lfsrNext(lfsr);
  r ^= (uint16_t)lfsr << 4;
  return 300u + (r % 1500u);
}

// -------- Button sense hybrid --------
static inline void buttonSense_Level() {
  PORTA.INTFLAGS = PIN4_bm;
  PORTA.PIN4CTRL = PORT_PULLUPEN_bm | PORT_ISC_LEVEL_gc;     // wake from Standby
  PORTA.INTFLAGS = PIN4_bm;
}
static inline void buttonSense_Falling() {
  PORTA.INTFLAGS = PIN4_bm;
  PORTA.PIN4CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;   // cheap while awake
  PORTA.INTFLAGS = PIN4_bm;
}
static inline void buttonSense_DisableButKeepPullup() {
  PORTA.INTFLAGS = PIN4_bm;
  PORTA.PIN4CTRL = PORT_PULLUPEN_bm | PORT_ISC_INTDISABLE_gc;
  PORTA.INTFLAGS = PIN4_bm;
}

// -------- RTC --------
static void rtcInit() {
  while (RTC.STATUS) {}
  RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;
  RTC.CTRLA  = RTC_RTCEN_bm | RTC_RUNSTDBY_bm | RTC_PRESCALER_DIV32768_gc; // 1 Hz
  RTC.INTCTRL  = RTC_OVF_bm;
  RTC.INTFLAGS = 0xFF;
}
static void rtcSchedule(uint16_t sec) {
  if (sec == 0) sec = 1;
  while (RTC.STATUS) {}
  RTC.CNT = 0;
  RTC.PER = sec - 1;
  RTC.INTFLAGS = RTC_OVF_bm;
}

// -------- ISRs --------
ISR(RTC_CNT_vect) {
  RTC.INTFLAGS = RTC_OVF_bm;
  intervalWake = true;
}
ISR(PORTA_PORT_vect) {
  uint8_t flags = PORTA.INTFLAGS;
  PORTA.INTFLAGS = flags;

  if (flags & PIN4_bm) {
    buttonSense_DisableButKeepPullup(); // prevent storms while held
    buttonWake = true;
  }
}

// -------- Explicit peripheral off --------
static void disableUnusedPeripherals() {
  ADC0.CTRLA = 0;   // ADC off
  AC0.CTRLA  = 0;   // comparator off
  VREF.CTRLA = 0;   // internal reference off
}

// -------- Clamp unused pins --------
static void clampUnusedPins() {
  // Used: PA4 button, PA7 LED pin (forced low)
  pinMode(PIN_PA0, OUTPUT); digitalWrite(PIN_PA0, LOW);
  pinMode(PIN_PA1, OUTPUT); digitalWrite(PIN_PA1, LOW);
  pinMode(PIN_PA2, OUTPUT); digitalWrite(PIN_PA2, LOW);
  pinMode(PA3_PAD, OUTPUT); digitalWrite(PA3_PAD, LOW); // important: don't float
  pinMode(PIN_PA5, OUTPUT); digitalWrite(PIN_PA5, LOW);
  pinMode(PIN_PA6, OUTPUT); digitalWrite(PIN_PA6, LOW);

  // Used: PB0/PB1 piezo (clamped low when idle)
  // Clamp: PB2/PB3
  pinMode(PIN_PB2, OUTPUT); digitalWrite(PIN_PB2, LOW);
  pinMode(PIN_PB3, OUTPUT); digitalWrite(PIN_PB3, LOW);
}

// -------- Sleep --------
static inline void sleepStandby() {
  ledKillForever();
  piezoClampLow();      // keep piezo discharged before sleep too
  buttonSense_Level();  // LEVEL only right before sleep

  set_sleep_mode(SLEEP_MODE_STANDBY);
  sleep_enable();
  sleep_cpu();
  sleep_disable();

  // Awake now -> cheap edge sense
  buttonSense_Falling();
}

// -------- Setup --------
void setup() {
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);
  delay(1000);
  digitalWrite(LEDPIN, LOW);
  delay(10);

  ledKillForever();

  pinMode(BUTTON, INPUT_PULLUP);
  piezoClampLow();      // start discharged

  wait_ms(200);

  clampUnusedPins();
  disableUnusedPeripherals();

  rtcInit();

  // Seed RNG
  lfsr ^= (uint8_t)(VPORTA.IN);

  buttonSense_Falling();
  rtcSchedule(nextIntervalSeconds_5to30min());

  sei();
}

// -------- Loop --------
void loop() {
  sleepStandby();

  if (intervalWake) {
    intervalWake = false;
    rtcSchedule(nextIntervalSeconds_5to30min());
    beep_ms(BEEP_MS_INTERVAL);
  }

  if (buttonWake) {
    buttonWake = false;

    if (digitalRead(BUTTON) == LOW) {
      beep_ms(BEEP_MS_BUTTON);
    }

    while (digitalRead(BUTTON) == LOW) {}
    wait_ms(30);
    buttonSense_Falling();
  }
}
