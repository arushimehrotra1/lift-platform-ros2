#pragma once
// ============================================================================
//  config.h : every pin and constant in one place. Edit this for your wiring.
// ============================================================================
#include <Arduino.h>

// ---- Cytron MD10C (PWM + DIR mode). #1 = LEFT actuator, #2 = RIGHT ----------
#define PIN_L_PWM        25
#define PIN_L_DIR        26
#define PIN_R_PWM        27
#define PIN_R_DIR        13
#define ACT_L_INVERT     0       // set 1 if "UP" retracts the left actuator
#define ACT_R_INVERT     0       // set 1 if "UP" retracts the right actuator

// ---- Actuator feedback pots (ADC1 pins only; wiper -> pin, ends -> 3V3/GND) -
#define PIN_L_POT        34
#define PIN_R_POT        35

// ---- Safety inputs, INPUT_PULLUP. Wire normally-closed to GND.  -------------
//      closed to GND = LOW = OK.   Open / pressed / wire cut = HIGH = TRIGGERED
#define PIN_ESTOP        4
#define PIN_LIMIT_UPPER  32
#define PIN_LIMIT_LOWER  33
#define USE_LIMIT_SWITCHES 1     // 0 only on the bench if switches aren't wired
#define TRIGGERED        HIGH

// ---- Motor drive -------------------------------------------------------------
#define PWM_FREQ_HZ      20000   // MD10C accepts up to 20 kHz
#define PWM_RES_BITS     10
#define RAMP_PER_SEC     2.5f    // soft start/stop: 0 -> 1.0 in 0.4 s
#define MIN_DRIVE        0.15f   // below this the actuator only hums
#define SERVICE_SPEED    0.50f   // speed used before calibration and for jogging

// ---- Timing ------------------------------------------------------------------
#define CONTROL_PERIOD_US   20000UL   // 50 Hz control loop
#define TELEMETRY_PERIOD_MS 50        // 20 Hz telemetry to the PC
#define MANUAL_HOLD_MS      400       // UP/DOWN/JOG must be refreshed within this

// ---- Pot reading -------------------------------------------------------------
#define POT_OVERSAMPLE   8
#define POT_FILTER_ALPHA 0.25f
#define RAW_FAULT_LOW    40      // wiper at a rail = broken wire
#define RAW_FAULT_HIGH   4055

// ---- Control defaults (changeable at runtime, saved to flash) -----------------
#define DEFAULT_STROKE_MM   100.0f    // ARF03-12-A1-205-100: 100 mm stroke
#define DEFAULT_KP          0.08f     // output per mm of height error
#define DEFAULT_KI          0.010f
#define DEFAULT_KD          0.000f
#define DEFAULT_KS          0.10f     // sync gain: output per mm of L-R mismatch
#define DEFAULT_VMAX        1.00f     // max common drive
#define DEADBAND_MM         1.0f      // "arrived" tolerance
#define INTEG_MAX_OUT       0.30f     // anti-windup: max contribution of I term
#define SYNC_MAX_CORR       0.40f
#define SOFT_MARGIN_MM      1.0f

// ---- Fault thresholds ---------------------------------------------------------
#define SYNC_FAULT_MM    12.0f
#define SYNC_FAULT_MS    500
#define JAM_MIN_CMD      0.30f
#define JAM_TIME_MS      1500
#define JAM_MIN_TRAVEL_MM 0.5f
