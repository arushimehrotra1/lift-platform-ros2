// ============================================================================
//  fw_sim : runs the REAL firmware (LiftPlatform.ino) on your PC against a
//  simulated pair of 7 mm/s actuators with potentiometers.
//     ./fw_sim --selftest   scripted closed-loop checks, runs in fast time
//     ./fw_sim --pty        real-time; prints a pty path to use as the serial
//                           port, so the ROS 2 stack can run with no hardware
// ============================================================================
#include <Arduino.h>
#include <Preferences.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <time.h>
#include <algorithm>
#include "../../firmware/LiftPlatform/LiftPlatform.ino"

uint32_t g_us = 0;
SerialStub Serial;

// ----- plant -----------------------------------------------------------------
static double px[2] = {10, 10};          // true extension, mm
static double gain[2] = {0.93, 1.07};    // actuators deliberately NOT identical
static double duty[2] = {0, 0};          // signed, from PWM + DIR
static int dirPin[2] = {0, 0};
static bool simEstop = false, rawBreak = false;
static const double SPEED = 7.0;         // mm/s at full duty (from the datasheet photo)

int analogRead(int pin) {                // L pot rises with extension, R pot is reversed
  if (rawBreak) return 0;
  return pin == PIN_L_POT ? (int)(400 + px[0] / 100.0 * 3200)
                          : (int)(3500 - px[1] / 100.0 * 3000);
}
int digitalRead(int pin) {
  if (pin == PIN_ESTOP) return simEstop ? HIGH : LOW;
  if (pin == PIN_LIMIT_UPPER) return std::max(px[0], px[1]) >= 100.0 ? HIGH : LOW;
  if (pin == PIN_LIMIT_LOWER) return std::min(px[0], px[1]) <= 0.0 ? HIGH : LOW;
  return LOW;
}
void digitalWrite(int pin, int v) {
  if (pin == PIN_L_DIR) dirPin[0] = v; if (pin == PIN_R_DIR) dirPin[1] = v;
}
void ledcWrite(int pin, uint32_t d) {
  int i = (pin == PIN_L_PWM) ? 0 : 1;
  double m = (double)d / ((1 << PWM_RES_BITS) - 1);
  duty[i] = dirPin[i] ? m : -m;
}
static void stepPlant(double dt) {
  for (int i = 0; i < 2; i++) {
    px[i] += duty[i] * SPEED * gain[i] * dt;
    px[i] = std::min(100.0, std::max(0.0, px[i]));
  }
}
// advance 1 ms at a time (firmware loop() is polled, like on the ESP32)
static void run(double seconds) {
  for (int n = 0; n < (int)(seconds * 1000); n++) { g_us += 1000; stepPlant(0.001); loop(); }
}
static std::string drain() { std::string s = Serial.tx; Serial.tx.clear(); return s; }
static void send(const char* c) { Serial.rx += c; Serial.rx += '\n'; }

// ----- self test ---------------------------------------------------------------
static int fails = 0;
#define CHECK(c, msg) do { bool ok_=(c); printf("  [%s] %s\n", ok_?"PASS":"FAIL", msg); if(!ok_) fails++; } while(0)
static double Hm() { return 0.5 * (px[0] + px[1]); }

static int selftest() {
  setup(); run(0.2); drain();
  printf("1. uncalibrated: GOTO must be refused\n");
  send("GOTO 50"); run(0.1);
  CHECK(drain().find("N,not_calibrated") != std::string::npos, "GOTO refused before calibration");

  printf("2. calibration (pots have opposite polarity)\n");
  px[0] = px[1] = 0;   run(0.2); send("CALMIN"); run(0.1);
  px[0] = px[1] = 100; run(0.2); send("CALMAX"); run(0.1);
  px[0] = px[1] = 10;  run(0.3); drain();
  send("GET"); run(0.1); std::string g = drain();
  CHECK(g.find("cal=1") != std::string::npos, "calibrated flag set");

  printf("3. GOTO 50 from 10 mm with mismatched actuators (x0.93 / x1.07)\n");
  send("GOTO 50"); double maxErr = 0; int t = 0;
  for (; t < 6000; t++) { run(0.01); maxErr = std::max(maxErr, fabs(px[0]-px[1]));
    if (drain().find("I,arrived") != std::string::npos) break; }
  printf("     arrived after %.1f s, H=%.2f mm, max |xL-xR|=%.2f mm\n", t*0.01, Hm(), maxErr);
  CHECK(fabs(Hm() - 50) < 1.5, "height within 1.5 mm of target");
  CHECK(maxErr < 4.0, "sync keeps mismatch under 4 mm");
  run(0.5); CHECK(fabs(duty[0]) < 1e-6 && fabs(duty[1]) < 1e-6, "motors idle after arrival");

  printf("4. GOTO 20 (down)\n");
  send("GOTO 20"); for (t = 0; t < 6000; t++) { run(0.01); if (drain().find("I,arrived") != std::string::npos) break; }
  CHECK(fabs(Hm() - 20) < 1.5, "height within 1.5 mm of 20");

  printf("5. hold-to-move: UP sent once, must stop by itself\n");
  double h0 = Hm(); send("UP"); run(2.0);
  CHECK(Hm() - h0 < 4.0 && fabs(duty[0]) < 1e-6, "stops by itself (coasts < 4 mm on the soft ramp)");

  printf("6. hold-to-move: keep refreshing UP for 3 s\n");
  h0 = Hm(); for (int k = 0; k < 30; k++) { send("UP"); run(0.1); }
  CHECK(Hm() - h0 > 10, "moves while commands keep coming");
  send("STOP"); run(1.0);

  printf("7. E-stop latches, RESET refused while pressed\n");
  send("GOTO 60"); run(1.0); drain();
  simEstop = true; run(0.1);
  CHECK(drain().find("E,estop") != std::string::npos, "estop fault raised");
  CHECK(fabs(duty[0]) < 1e-6 && fabs(duty[1]) < 1e-6, "motors stopped at once");
  send("RESET"); run(0.1); CHECK(drain().find("N,estop_still_pressed") != std::string::npos, "RESET refused");
  simEstop = false; send("UP"); run(0.1); CHECK(drain().find("N,fault_latched") != std::string::npos, "still latched after release");
  send("RESET"); run(0.1); CHECK(drain().find("A,reset") != std::string::npos, "RESET accepted after release");

  printf("8. jammed actuator (right side stuck) -> latched fault\n");
  gain[1] = 0; send("GOTO 90"); drain(); bool faulted = false;
  for (t = 0; t < 1000 && !faulted; t++) { run(0.01); std::string s = drain();
    faulted = s.find("E,jam") != std::string::npos || s.find("E,sync_mismatch") != std::string::npos; }
  CHECK(faulted, "jam/sync fault raised"); printf("     after %.1f s\n", t*0.01);
  CHECK(fabs(duty[0]) < 1e-6, "healthy side stopped too");
  gain[1] = 1.07; send("RESET"); run(0.1); drain();

  printf("9. recovery jog of one actuator\n");
  px[0] = 60; px[1] = 40; run(0.1);
  double r0 = px[1]; for (int k = 0; k < 25; k++) { send("JOG R 1"); run(0.1); }
  CHECK(px[1] - r0 > 3 && fabs(px[0] - 60) < 0.5, "only the right actuator moved");
  send("STOP"); run(0.5); send("RESET"); run(0.1); drain();

  printf("10. pot wire break\n");
  px[0] = px[1] = 50; run(0.3); rawBreak = true; run(0.3);
  CHECK(drain().find("E,pot_wire") != std::string::npos, "pot fault raised");
  rawBreak = false;

  printf("\n%s (%d failure%s)\n", fails ? "SELF-TEST FAILED" : "SELF-TEST PASSED", fails, fails==1?"":"s");
  return fails ? 1 : 0;
}

// ----- real-time pty mode ---------------------------------------------------------
static int ptyMode() {
  int m = posix_openpt(O_RDWR | O_NOCTTY); grantpt(m); unlockpt(m);
  { termios t; tcgetattr(m, &t); cfmakeraw(&t); tcsetattr(m, TCSANOW, &t); }   // no echo / line editing
  { int sl = open(ptsname(m), O_RDWR | O_NOCTTY); termios t; tcgetattr(sl, &t); cfmakeraw(&t); tcsetattr(sl, TCSANOW, &t); /* keep slave open */ }
  printf("PTY %s\n", ptsname(m)); fflush(stdout);
  px[0] = px[1] = 0;
  setup();
  timespec ts{0, 1000000}; // 1 ms
  for (;;) {
    g_us += 1000; stepPlant(0.001); loop();
    if (!Serial.tx.empty()) { if (write(m, Serial.tx.data(), Serial.tx.size()) < 0) {} Serial.tx.clear(); }
    pollfd p{m, POLLIN, 0};
    if (poll(&p, 1, 0) > 0 && (p.revents & POLLIN)) { char b[256]; ssize_t n = read(m, b, sizeof b); if (n > 0) Serial.rx.append(b, n); }
    nanosleep(&ts, nullptr);
  }
}

int main(int argc, char** argv) {
  if (argc > 1 && !strcmp(argv[1], "--pty")) return ptyMode();
  return selftest();
}
