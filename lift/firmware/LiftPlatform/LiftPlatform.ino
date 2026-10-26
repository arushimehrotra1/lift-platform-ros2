// ============================================================================
//  Portable Powered Lifting Platform : ESP32 firmware
//
//  Closed loop:  height PID (outer)  ->  actuator sync (inner)  ->  MD10C PWM
//  Feedback   :  one potentiometer per actuator (mean extension = height)
//  Interface  :  line-based text over USB serial (see README), 115200 baud
//
//  Safety: faults LATCH and stop both motors until RESET. The E-stop must ALSO
//  cut motor power in hardware; this firmware is not a substitute for that.
// ============================================================================
#include <Preferences.h>
#include "config.h"

enum State : uint8_t { ST_IDLE = 0, ST_MANUAL, ST_GOTO, ST_JOG, ST_FAULT };
enum Fault : uint8_t { F_NONE = 0, F_ESTOP, F_POT, F_SYNC, F_JAM };

// ------------------------------- settings -----------------------------------
struct Settings {
  uint32_t magic;
  float raw0[2], raw1[2];            // pot raw at 0 mm and at full stroke (L, R)
  float kp, ki, kd, ks, vmax, stroke;
  uint8_t calMin, calMax;
};
static const uint32_t MAGIC = 0x4C494654;
static Settings S;
static Preferences prefs;

static void settingsDefaults() {
  S.magic = MAGIC;
  for (int i = 0; i < 2; i++) { S.raw0[i] = 300; S.raw1[i] = 3800; }
  S.kp = DEFAULT_KP; S.ki = DEFAULT_KI; S.kd = DEFAULT_KD; S.ks = DEFAULT_KS;
  S.vmax = DEFAULT_VMAX; S.stroke = DEFAULT_STROKE_MM;
  S.calMin = S.calMax = 0;
}
static void settingsLoad() {
  settingsDefaults();
  Settings t;
  prefs.begin("lift", true);
  size_t n = prefs.getBytes("s", &t, sizeof(t));
  prefs.end();
  if (n == sizeof(t) && t.magic == MAGIC) S = t;
}
static void settingsSave() {
  prefs.begin("lift", false);
  prefs.putBytes("s", &S, sizeof(S));
  prefs.end();
}
static bool calibrated() {
  return S.calMin && S.calMax &&
         fabsf(S.raw1[0] - S.raw0[0]) > 500 && fabsf(S.raw1[1] - S.raw0[1]) > 500;
}

// -------------------------------- globals -----------------------------------
static State  state = ST_IDLE;
static Fault  fault = F_NONE;
static float  x[2] = {0, 0};          // extension per actuator, mm
static float  rawF[2] = {0, 0};       // filtered pot raw
static bool   potBad = false, firstRead = true;
static float  H = 0, prevH = 0;       // platform height proxy = mean extension, mm
static float  target = 0, integ = 0;
static uint32_t arrivedMs = 0;
static float  uCmd[2] = {0, 0};       // commanded (before ramp)
static float  uOut[2] = {0, 0};       // applied (after ramp)
static int8_t manualDir = 0;          // +1 up, -1 down
static int8_t jogAct = 0, jogDir = 0; // single-actuator jog
static uint32_t lastCmdMs = 0;
static uint32_t syncBadMs = 0;
static float  jamRef[2] = {0, 0};
static uint32_t jamMs[2] = {0, 0};
static uint32_t lastTick = 0, lastTelem = 0;
static bool   limUp = false, limLo = false, estop = false;

static void say(const char* s) { Serial.println(s); }
static void sayf(const char* tag, const char* msg) {
  char b[96]; snprintf(b, sizeof(b), "%s,%s", tag, msg); Serial.println(b);
}

// --------------------------------- motors -----------------------------------
static void motorWrite(int i, float u) {
  u = constrain(u, -1.0f, 1.0f);
  bool up = (u >= 0);
  if (i == 0 ? ACT_L_INVERT : ACT_R_INVERT) up = !up;
  digitalWrite(i == 0 ? PIN_L_DIR : PIN_R_DIR, up ? HIGH : LOW);
  ledcWrite(i == 0 ? PIN_L_PWM : PIN_R_PWM,
            (uint32_t)(fabsf(u) * ((1 << PWM_RES_BITS) - 1)));
}
static void motorsStopNow() {
  for (int i = 0; i < 2; i++) { uCmd[i] = uOut[i] = 0; motorWrite(i, 0); }
}
static void applyOutputs(float dt) {
  for (int i = 0; i < 2; i++) {
    float u = uCmd[i], a = fabsf(u);
    if (a < 0.05f) u = 0;                                   // too small to matter
    else if (a < MIN_DRIVE) u = (u > 0 ? MIN_DRIVE : -MIN_DRIVE);
    float step = RAMP_PER_SEC * dt;                          // soft start/stop
    if (u > uOut[i] + step) uOut[i] += step;
    else if (u < uOut[i] - step) uOut[i] -= step;
    else uOut[i] = u;
    motorWrite(i, uOut[i]);
  }
}

// ------------------------------ sensing -------------------------------------
static void readPots() {
  const int pins[2] = {PIN_L_POT, PIN_R_POT};
  potBad = false;
  for (int i = 0; i < 2; i++) {
    long sum = 0;
    for (int k = 0; k < POT_OVERSAMPLE; k++) sum += analogRead(pins[i]);
    float raw = (float)sum / POT_OVERSAMPLE;
    if (raw < RAW_FAULT_LOW || raw > RAW_FAULT_HIGH) potBad = true;
    rawF[i] = firstRead ? raw : rawF[i] + POT_FILTER_ALPHA * (raw - rawF[i]);
    float span = S.raw1[i] - S.raw0[i];
    x[i] = (span == 0) ? 0 : (rawF[i] - S.raw0[i]) / span * S.stroke;   // works for either pot polarity
  }
  firstRead = false;
  H = 0.5f * (x[0] + x[1]);
}

// Fresh, heavily averaged raw reading (used for calibration; ignores the filter lag)
static float readRawAvg(int i, int n) {
  long sum = 0;
  for (int k = 0; k < n; k++) sum += analogRead(i == 0 ? PIN_L_POT : PIN_R_POT);
  return (float)sum / n;
}

// ------------------------------- safety -------------------------------------
static void trip(Fault f) {
  fault = f; state = ST_FAULT; motorsStopNow();
  static const char* names[] = {"none", "estop", "pot_wire", "sync_mismatch", "jam"};
  sayf("E", names[f]);
}
static void safetyCheck(uint32_t now) {
  if (state == ST_FAULT) return;
  if (estop)  { trip(F_ESTOP); return; }
  if (potBad) { trip(F_POT);   return; }
  if (!calibrated()) return;
  // sync mismatch (not while the operator is deliberately jogging one side)
  if (state != ST_JOG && fabsf(x[0] - x[1]) > SYNC_FAULT_MM) {
    if (syncBadMs == 0) syncBadMs = now;
    if (now - syncBadMs > SYNC_FAULT_MS) { trip(F_SYNC); return; }
  } else syncBadMs = 0;
  // jam: driven hard but not moving
  for (int i = 0; i < 2; i++) {
    if (fabsf(uOut[i]) > JAM_MIN_CMD) {
      if (jamMs[i] == 0) { jamMs[i] = now; jamRef[i] = x[i]; }
      if (fabsf(x[i] - jamRef[i]) > JAM_MIN_TRAVEL_MM) { jamMs[i] = now; jamRef[i] = x[i]; }
      else if (now - jamMs[i] > JAM_TIME_MS) { trip(F_JAM); return; }
    } else jamMs[i] = 0;
  }
}

// ------------------------------- control ------------------------------------
// Height PID with conditional-integration anti-windup. Returns common command u0.
static float heightPID(float dt) {
  float err = target - H;
  if (fabsf(err) < DEADBAND_MM) {
    integ = 0;
    arrivedMs += (uint32_t)(dt * 1000);
    if (arrivedMs > 300) { state = ST_IDLE; sayf("I", "arrived"); }
    return 0;
  }
  arrivedMs = 0;
  float d = -(H - prevH) / dt;                       // derivative on measurement
  float uRaw = S.kp * err + S.ki * integ + S.kd * d;
  float u = constrain(uRaw, -S.vmax, S.vmax);
  if (u == uRaw) {                                   // integrate only when not saturated
    integ += err * dt;
    float lim = (S.ki > 1e-6f) ? INTEG_MAX_OUT / S.ki : 0;
    integ = constrain(integ, -lim, lim);
  }
  return u;
}

// Inner loop:  uL = u0 - Ks*e,  uR = u0 + Ks*e,  e = xL - xR.
// The leader is slowed but never reversed; the follower is sped up.
static void syncSplit(float u0, float* uL, float* uR) {
  float corr = constrain(S.ks * (x[0] - x[1]), -SYNC_MAX_CORR, SYNC_MAX_CORR);
  *uL = u0 - corr;
  *uR = u0 + corr;
  if (u0 > 0) { *uL = constrain(*uL, 0.f, 1.f);  *uR = constrain(*uR, 0.f, 1.f); }
  if (u0 < 0) { *uL = constrain(*uL, -1.f, 0.f); *uR = constrain(*uR, -1.f, 0.f); }
}

static bool blockedUp(int i)   { return limUp || (calibrated() && x[i] >= S.stroke - SOFT_MARGIN_MM); }
static bool blockedDown(int i) { return limLo || (calibrated() && x[i] <= SOFT_MARGIN_MM); }

static void controlTick(float dt, uint32_t now) {
  readPots();
  estop = digitalRead(PIN_ESTOP) == TRIGGERED;
#if USE_LIMIT_SWITCHES
  limUp = digitalRead(PIN_LIMIT_UPPER) == TRIGGERED;
  limLo = digitalRead(PIN_LIMIT_LOWER) == TRIGGERED;
#endif
  safetyCheck(now);

  if (state == ST_FAULT) { motorsStopNow(); prevH = H; return; }

  // operator hold-to-move timeout
  if ((state == ST_MANUAL || state == ST_JOG) && now - lastCmdMs > MANUAL_HOLD_MS) {
    state = ST_IDLE; manualDir = jogDir = 0;
  }

  float uL = 0, uR = 0;
  if (state == ST_JOG) {
    float u = jogDir * SERVICE_SPEED;
    int i = jogAct;
    if ((u > 0 && blockedUp(i)) || (u < 0 && blockedDown(i))) u = 0;
    (i == 0 ? uL : uR) = u;                                   // no sync while jogging
  } else if (state == ST_MANUAL || state == ST_GOTO) {
    float u0 = 0;
    if (state == ST_MANUAL) u0 = manualDir * (calibrated() ? S.vmax : SERVICE_SPEED);
    else u0 = heightPID(dt);
    // limits act on the common command so both sides stop together
    bool up = u0 > 0, dn = u0 < 0;
    if ((up && (blockedUp(0) || blockedUp(1))) || (dn && (blockedDown(0) || blockedDown(1)))) {
      u0 = 0;
      if (state == ST_GOTO) { state = ST_IDLE; sayf("I", "limit_reached"); }
    }
    if (calibrated()) syncSplit(u0, &uL, &uR);
    else uL = uR = u0;                                        // uncalibrated: no sync
  }
  uCmd[0] = uL; uCmd[1] = uR;
  prevH = H;
  applyOutputs(dt);
}

// ------------------------------ commands ------------------------------------
static void reply(bool ok, const char* msg) { sayf(ok ? "A" : "N", msg); }

static void doReset() {
  if (digitalRead(PIN_ESTOP) == TRIGGERED) { reply(false, "estop_still_pressed"); return; }
  fault = F_NONE; state = ST_IDLE; integ = 0; syncBadMs = 0;
  jamMs[0] = jamMs[1] = 0; motorsStopNow(); reply(true, "reset");
}

static void handleLine(char* line) {
  char* cmd = strtok(line, " ,");
  if (!cmd) return;
  char* a1 = strtok(NULL, " ,");
  char* a2 = strtok(NULL, " ,");
  char* a3 = strtok(NULL, " ,");
  uint32_t now = millis();

  if (!strcmp(cmd, "STOP")) {
    if (state != ST_FAULT) state = ST_IDLE;
    manualDir = jogDir = 0; motorsStopNow(); reply(true, "stop");
  } else if (!strcmp(cmd, "RESET")) {
    doReset();
  } else if (!strcmp(cmd, "UP") || !strcmp(cmd, "DOWN")) {
    if (state == ST_FAULT) { reply(false, "fault_latched"); return; }
    if (state != ST_MANUAL) integ = 0;
    state = ST_MANUAL; manualDir = (cmd[0] == 'U') ? 1 : -1; lastCmdMs = now;
  } else if (!strcmp(cmd, "JOG") && a1 && a2) {       // JOG L|R 1|-1  (sync-fault recovery)
    if (estop) { reply(false, "estop"); return; }
    if (state == ST_FAULT) { fault = F_NONE; state = ST_IDLE; motorsStopNow(); }
    jogAct = (a1[0] == 'R') ? 1 : 0; jogDir = atoi(a2) >= 0 ? 1 : -1;
    state = ST_JOG; lastCmdMs = now;
  } else if (!strcmp(cmd, "GOTO") && a1) {
    if (state == ST_FAULT) { reply(false, "fault_latched"); return; }
    if (!calibrated())     { reply(false, "not_calibrated"); return; }
    target = constrain((float)atof(a1), 0.0f, S.stroke);
    integ = 0; arrivedMs = 0; state = ST_GOTO; reply(true, "goto");
  } else if (!strcmp(cmd, "CALMIN") || !strcmp(cmd, "CALMAX")) {
    if (state != ST_IDLE) { reply(false, "must_be_idle"); return; }
    bool mn = !strcmp(cmd, "CALMIN");
    for (int i = 0; i < 2; i++) (mn ? S.raw0[i] : S.raw1[i]) = readRawAvg(i, 64);
    (mn ? S.calMin : S.calMax) = 1;
    settingsSave(); reply(true, mn ? "calmin" : "calmax");
  } else if (!strcmp(cmd, "PID") && a1 && a2 && a3) {
    S.kp = atof(a1); S.ki = atof(a2); S.kd = atof(a3); settingsSave(); reply(true, "pid");
  } else if (!strcmp(cmd, "KS") && a1) {
    S.ks = atof(a1); settingsSave(); reply(true, "ks");
  } else if (!strcmp(cmd, "VMAX") && a1) {
    S.vmax = constrain((float)atof(a1), 0.2f, 1.0f); settingsSave(); reply(true, "vmax");
  } else if (!strcmp(cmd, "GET")) {
    char b[128];
    snprintf(b, sizeof(b), "S,kp=%.4f,ki=%.4f,kd=%.4f,ks=%.3f,vmax=%.2f,stroke=%.1f,cal=%d",
             S.kp, S.ki, S.kd, S.ks, S.vmax, S.stroke, (int)calibrated());
    say(b);
  } else {
    reply(false, "unknown_cmd");
  }
}

static void pollSerial() {
  static char buf[64]; static uint8_t n = 0;
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c < 0) break;
    if (c == '\n' || c == '\r') { if (n) { buf[n] = 0; handleLine(buf); n = 0; } }
    else if (n < sizeof(buf) - 1) buf[n++] = (char)c;
    else n = 0;                                    // overlong line: drop it
  }
}

// Telemetry: T,ms,xL,xR,H,target,uL,uR,state,fault,calibrated
static void telemetry(uint32_t now) {
  char b[128];
  snprintf(b, sizeof(b), "T,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d",
           (unsigned long)now, x[0], x[1], H, target, uOut[0], uOut[1],
           (int)state, (int)fault, (int)calibrated());
  Serial.println(b);
}

// --------------------------------- Arduino ----------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_L_DIR, OUTPUT); pinMode(PIN_R_DIR, OUTPUT);
  digitalWrite(PIN_L_DIR, LOW); digitalWrite(PIN_R_DIR, LOW);
  ledcAttach(PIN_L_PWM, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttach(PIN_R_PWM, PWM_FREQ_HZ, PWM_RES_BITS);
  motorsStopNow();
  pinMode(PIN_ESTOP, INPUT_PULLUP);
  pinMode(PIN_LIMIT_UPPER, INPUT_PULLUP); pinMode(PIN_LIMIT_LOWER, INPUT_PULLUP);
  analogReadResolution(12);
  settingsLoad();
  readPots(); prevH = H; target = H;
  say("I,boot_ok");
}

void loop() {
  pollSerial();
  uint32_t nowUs = micros();
  if (nowUs - lastTick >= CONTROL_PERIOD_US) {
    float dt = (nowUs - lastTick) * 1e-6f;
    lastTick = nowUs;
    if (dt > 0.1f) dt = 0.02f;                     // first tick / long stall
    controlTick(dt, millis());
  }
  uint32_t now = millis();
  if (now - lastTelem >= TELEMETRY_PERIOD_MS) { lastTelem = now; telemetry(now); }
}
