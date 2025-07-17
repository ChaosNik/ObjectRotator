/****************************************************************************
 *  ObjectRotator.cpp  –  simple “ZERO_DELTA_DEG-only” homing
 *
 *  • Homes to the hard stop at power‑up and at the end of every preset cycle
 *  • After the switch trips, it backs off a hair (internal constant) and
 *    then moves ZERO_DELTA_DEG so the shaft sits at POS_DEG[0]
 *  • All angles in POS_DEG[] and ZERO_DELTA_DEG are OUTPUT‑SHAFT degrees
 *  • ***2025‑07‑16: limit switch converted to NORMALLY‑CLOSED (active‑HIGH)***
 *
 *  Author : Nikola Karpić <nikola_karpic@live.com>
 *  Created: 2025‑04‑27
 *  Revised: 2025‑07‑17
 *  License: MIT
 ****************************************************************************/

 #include <Stepper.h>

 /* ── user‑tunable numbers (OUTPUT degrees) ──────────────────────────────── */
 const float  POS_DEG[] = { -30, 0, 30 };     // list of output angles
 const int    SPEED_RPM  = 10;                // normal running speed
 const int    ZERO_DELTA_DEG = 5;             // ° between released switch and
                                              //    first logical preset
 // const float  GEAR_MULT  = 9.111;          // motor ° per 1 ° output
 const float  GEAR_MULT  = 1.0f;              // motor ° per 1 ° output TESTING
 
 /* ── motor wiring ───────────────────────────────────────────────────────── */
 const int  STEPS_PER_REV = 2038;             // 28BYJ‑48 w/ built‑in gearbox
 const int  IN1 = 8, IN3 = 10, IN2 = 9, IN4 = 11;
 Stepper     motor(STEPS_PER_REV, IN1, IN3, IN2, IN4);
 
 /* (optional) internal micro‑offset to release the lever */
 const int   RELEASE_DEG = 2;                 // leave it small & fixed
 
 /* ── I/O pins & timing ──────────────────────────────────────────────────── */
 const int      BTN_PIN      = 2;
 const int      LIM_PIN      = 3;             // NC limit switch → active‑HIGH
 const uint8_t  LED_PIN      = 13;
 const uint8_t  DEBOUNCE_MS  = 50;
 const uint16_t LONGPRESS_MS = 3000;
 
 /* ── preset list (OUTPUT angles) ────────────────────────────────────────── */
 const uint8_t POS_CNT  = sizeof(POS_DEG) / sizeof(POS_DEG[0]);
 
 /* ── derived constants ──────────────────────────────────────────────────── */
 const float STEPS_PER_DEG_OUT =
         (float)STEPS_PER_REV / 360.0f * GEAR_MULT;
 
 /* ── state ─────────────────────────────────────────────────────────────── */
 float   currentDeg  = POS_DEG[0];
 uint8_t posIdx      = 0;
 
 bool    lastBtn         = HIGH;
 unsigned long tEdge     = 0;
 bool    longPressActive = false;
 
 /* ───────────────── helper routines ─────────────────────────────────────── */
 
 void zeroHere() {
   currentDeg = POS_DEG[0];
   posIdx     = 0;
 }
 
 /* Crawl to switch, release, jog ZERO_DELTA_DEG, resync logic */
 void homeLeftUntilLimit() {
 
   /* crawl left until switch OPENS (active‑HIGH NC) */
   while (digitalRead(LIM_PIN) == LOW)        // ← changed for NC
     motor.step(-10);                         // small negative steps
 
   /* quick lever release (internal constant) */
   motor.step( (long)(RELEASE_DEG * STEPS_PER_DEG_OUT) );
 
   /* jog ZERO_DELTA_DEG so we’re at the first preset */
   motor.step( (long)(ZERO_DELTA_DEG  * STEPS_PER_DEG_OUT) );
 
   zeroHere();
 }
 
 /* advance to next preset; wrap triggers homing */
 void moveNextPosition() {
   uint8_t nextIdx = (posIdx + 1) % POS_CNT;
 
   if (nextIdx == 0) {                      // wrapped → home
     homeLeftUntilLimit();
     return;
   }
 
   float target = POS_DEG[nextIdx];
   float delta  = target - currentDeg;
   if (delta >  180) delta -= 360;
   if (delta < -180) delta += 360;
 
   motor.step( delta * STEPS_PER_DEG_OUT );
 
   currentDeg = target;
   posIdx     = nextIdx;
 }
 
 /* ───────────────── Arduino setup / loop ────────────────────────────────── */
 
 void setup() {
   motor.setSpeed(SPEED_RPM);
   pinMode(BTN_PIN, INPUT_PULLUP);
   pinMode(LIM_PIN, INPUT_PULLUP);
   pinMode(LED_PIN, OUTPUT);
 
   homeLeftUntilLimit();                    // initial home
 }
 
 void loop() {
   bool btn = digitalRead(BTN_PIN);
   bool lim = (digitalRead(LIM_PIN) == HIGH);   // ← active when HIGH (NC)
 
   /* debounce */
   static bool btnLatched = false;
   if (btn != lastBtn) { lastBtn = btn; tEdge = millis(); }
   unsigned long held = millis() - tEdge;
 
   /* long‑press = free‑spin left */
   if (btn == LOW && held >= LONGPRESS_MS && !longPressActive) {
     longPressActive = true;
     zeroHere();
   }
   if (btn == HIGH && longPressActive) longPressActive = false;
 
   /* free‑spin behaviour */
   if (longPressActive && !lim) {           // spin until switch triggers
     motor.step(-10);
     return;
   }
 
   /* short press → advance / home */
   if (btn == LOW && held > DEBOUNCE_MS && !btnLatched) {
     moveNextPosition();
     btnLatched = true;
   }
   if (btn == HIGH) btnLatched = false;
 
   /* LED shows switch state only (optional) */
   digitalWrite(LED_PIN, lim ? HIGH : LOW);
 }
 