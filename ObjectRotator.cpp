/******************************************************************************
 *  File:        ObjectRotator.cpp
 *  Purpose:     Drive a 28BYJ-48 stepper via Arduino Stepper library
 *
 *  Author:      Nikola Karpić <nikola_karpic@live.com>
 *  Created:     2025-04-27
 *
 *  License:     MIT
 ******************************************************************************/

#include <Stepper.h>

/* ── motor setup ──────────────────────────────── */
const int  STEPS_PER_REV = 2038;                 // 28BYJ-48 + gearbox
const int  IN1 = 8, IN3 = 10, IN2 = 9, IN4 = 11;
const float STEPS_PER_DEG = STEPS_PER_REV / 360.0;
Stepper motor(STEPS_PER_REV, IN1, IN3, IN2, IN4);

/* ── user settings ────────────────────────────── */
const int SPEED_RPM = 10;

/* ── I/O pins & timing ────────────────────────── */
const int      BTN_PIN      = 2;                 // momentary push-button
const int      LIM_PIN      = 3;                 // limit / home switch
const uint8_t  LED_PIN      = 13;                 // LED
const uint8_t  DEBOUNCE_MS  = 50;
const uint16_t LONGPRESS_MS = 3000;

/* ── position list (edit to suit) ─────────────── */
const float POS_DEG[] = { -30, 0, 30 }; // list of angles
const uint8_t POS_CNT = sizeof(POS_DEG) / sizeof(POS_DEG[0]);

/* ── state - don’t touch ──────────────────────── */
float    currentDeg = POS_DEG[0];                // logical angle
uint8_t  posIdx     = 0;                         // index in POS_DEG[]

bool     lastBtn          = HIGH;               // pull-up → idle HIGH
unsigned long tStateChange = 0;
bool     longPressActive  = false;

bool     limitLatched = false;                  // fire once per hit

/* ─────────────────────────────────────────────── */
void setup() {
  motor.setSpeed(SPEED_RPM);
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(LIM_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  /* ---------- read inputs ---------- */
  bool btn = digitalRead(BTN_PIN);
  bool lim = (digitalRead(LIM_PIN)   == LOW);

  /* ---------- debounce / timing ---------- */
  if (btn != lastBtn) {                         // any edge on button
    lastBtn = btn;
    tStateChange = millis();
  }
  unsigned long heldMs = millis() - tStateChange;

  /* ---------- enter spin mode on long-press ---------- */
  if (btn == LOW && heldMs >= LONGPRESS_MS && !longPressActive) {
    longPressActive = true;
    zeroHere();                                 // **re-zero while holding**
  }

  /* ---------- leave spin mode when released ---------- */
  if (btn == HIGH && longPressActive) {
    longPressActive = false;
  }

  /* ---------- fire limit switch any time ---------- */
  if (lim && !limitLatched) {                   // first contact
    zeroHere();                                 // **instant home**
    limitLatched = true;
    digitalWrite(LED_PIN, HIGH);  // turn it on
  }
  if (!lim) {
    digitalWrite(LED_PIN, LOW);  // turn it off
    limitLatched = false;               // ready for next hit
  }

  /* ---------- behaviour while spinning ---------- */
  if (longPressActive && !lim) {
    motor.step(-10);                             // free-spin
    return;                                     // ignore short-press logic
  }

  /* ---------- single short press = next preset ---------- */
  static bool btnLatched = false;
  if (btn == LOW && heldMs > DEBOUNCE_MS && !btnLatched) {
    moveNextPosition();
    btnLatched = true;
  }
  if (btn == HIGH) btnLatched = false;
}

/* ---------------------------------------------------------
   Centre-of-sketch helpers
----------------------------------------------------------*/
void zeroHere() {
  currentDeg = POS_DEG[0];
  posIdx     = 0;
}

void moveNextPosition() {
  posIdx = (posIdx + 1) % POS_CNT;              // advance & wrap
  float targetDeg = POS_DEG[posIdx];

  /* shortest path around the circle */
  float deltaDeg = targetDeg - currentDeg;
  if (deltaDeg > 180)  deltaDeg -= 360;
  if (deltaDeg < -180) deltaDeg += 360;

  motor.step(deltaDeg * STEPS_PER_DEG);
  currentDeg = targetDeg;
}
