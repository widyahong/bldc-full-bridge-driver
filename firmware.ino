// Open-loop BLDC, 3 half-bridges (6 gate inputs), 6-step trapezoidal, LEDC with software dead-time.
// Target: ESP32 (Arduino-ESP32 core 2.x or 3.x), no external libraries (uses ESP-IDF driver/ledc.h).
// Serial 115200, commands end with newline:
//   d<0..100> duty %      m<10..98> max duty %   t<0..5000> dead-time ns
//   f<1..500> elec Hz     p<1000..40000> PWM Hz  r<ms> ramp time
//   x0/x1 direction       e0/e1 disable/enable   ? status

#include <Arduino.h>
#include "driver/ledc.h"

// ---------------- User settings (defaults) ----------------
// Order: AH, AL, BH, BL, CH, CL  (all assumed active-high at the driver input)
static const int8_t GATE_PIN[6] = {25, 26, 27, 32, 33, 23};

uint32_t PWM_FREQ_HZ        = 20000;
float    DUTY_PERCENT       = 40.0f;
float    MAX_DUTY_PERCENT   = 95.0f;
uint32_t DEAD_TIME_NS       = 500;
float    ELEC_FREQ_HZ       = 20.0f;
uint32_t RAMP_TIME_MS       = 2000;
float    START_FREQ_HZ      = 3.0f;
float    START_DUTY_PERCENT = 20.0f;
uint32_t BOOT_CHARGE_MS     = 20;      // all low-sides ON before start (bootstrap charge)
uint8_t  DIRECTION          = 0;       // 0 forward, 1 reverse
bool     ENABLE_ON_BOOT     = false;

// ---------------- Internals ----------------
static const ledc_mode_t  LEDC_MODE  = LEDC_LOW_SPEED_MODE;
static const ledc_timer_t LEDC_TMR   = LEDC_TIMER_0;

// 6-step table: {PWM (high) phase, return (low ON) phase}, phase 0=A 1=B 2=C
static const uint8_t SEQ[6][2] = {{0, 1}, {0, 2}, {1, 2}, {1, 0}, {2, 0}, {2, 1}};

uint8_t  pwmBits   = 11;
uint32_t pwmPeriod = 2048;
uint32_t deadTicks = 0;

bool     enabled       = false;
bool     needApply     = false;
uint8_t  stepIdx       = 0;
uint32_t lastStepUs    = 0;
uint32_t stepPeriodUs  = 1000000;
uint32_t runStartMs    = 0;
uint32_t lastRampMs    = 0;
uint32_t curD          = 0;            // current PWM duty in ticks

enum : uint8_t { CH_OFF = 0, CH_ON = 1, CH_PWM = 2 };
struct ChState { uint8_t mode; uint32_t duty; uint32_t hp; bool valid; };
ChState cache[6];

// ---- LEDC timer (resolution chosen from PWM frequency, dead-time converted to ticks) ----
bool timerConfig() {
  uint32_t ratio = 80000000UL / PWM_FREQ_HZ;
  int bits = 31 - __builtin_clz(ratio);
  if (bits < 8)  bits = 8;
  if (bits > 14) bits = 14;

  ledc_timer_config_t t = {};
  t.speed_mode      = LEDC_MODE;
  t.duty_resolution = (ledc_timer_bit_t)bits;
  t.timer_num       = LEDC_TMR;
  t.freq_hz         = PWM_FREQ_HZ;
  t.clk_cfg         = LEDC_AUTO_CLK;
  if (ledc_timer_config(&t) != ESP_OK) return false;

  pwmBits   = (uint8_t)bits;
  pwmPeriod = 1u << bits;
  deadTicks = (uint32_t)((double)DEAD_TIME_NS * PWM_FREQ_HZ * pwmPeriod / 1e9 + 0.5);
  if (deadTicks > pwmPeriod / 8) deadTicks = pwmPeriod / 8;
  return true;
}

// ---- Channel control with state cache (only touches LEDC when something changed) ----
void setCh(uint8_t ch, uint8_t mode, uint32_t duty = 0, uint32_t hp = 0) {
  ChState &c = cache[ch];
  if (c.valid && c.mode == mode && (mode != CH_PWM || (c.duty == duty && c.hp == hp))) return;
  ledc_channel_t lc = (ledc_channel_t)ch;
  if (mode == CH_OFF) {
    ledc_stop(LEDC_MODE, lc, 0);
  } else if (mode == CH_ON) {
    ledc_stop(LEDC_MODE, lc, 1);
  } else {
    ledc_set_duty_with_hpoint(LEDC_MODE, lc, duty, hp);
    ledc_update_duty(LEDC_MODE, lc);
  }
  c.mode = mode; c.duty = duty; c.hp = hp; c.valid = true;
}

void allOff() {
  for (uint8_t i = 0; i < 6; i++) setCh(i, CH_OFF);
}

// ---- Apply one commutation step ----
void applyOutput(uint8_t step, uint32_t D) {
  if (D == 0) { allOff(); return; }
  uint8_t hi = SEQ[step][0];
  uint8_t lo = SEQ[step][1];

  for (uint8_t p = 0; p < 3; p++) {
    uint8_t H = 2 * p, L = 2 * p + 1;
    if (p == lo) {                         // return phase: high OFF first, then low ON
      setCh(H, CH_OFF);
      setCh(L, CH_ON);
    } else if (p == hi) {                  // PWM phase: complementary with dead-time
      int32_t  lowDuty = (int32_t)pwmPeriod - (int32_t)D - 2 * (int32_t)deadTicks;
      uint32_t lowHp   = D + deadTicks;
      if (lowDuty < 1) {                   // no room for low-side window
        setCh(L, CH_OFF);
        setCh(H, CH_PWM, D, 0);
      } else if (cache[H].mode == CH_PWM && D < cache[H].duty) {
        setCh(H, CH_PWM, D, 0);            // duty decreasing: shorten high first
        setCh(L, CH_PWM, (uint32_t)lowDuty, lowHp);
      } else {
        setCh(L, CH_PWM, (uint32_t)lowDuty, lowHp);   // duty increasing: move low first
        setCh(H, CH_PWM, D, 0);
      }
    } else {                               // floating phase
      setCh(H, CH_OFF);
      setCh(L, CH_OFF);
    }
  }
}

// ---- Soft-start ramp (called every 1 ms) ----
void updateRamp(uint32_t nowMs) {
  uint32_t elapsed = nowMs - runStartMs;
  float k  = (RAMP_TIME_MS == 0) ? 1.0f : min(1.0f, elapsed / (float)RAMP_TIME_MS);
  float f0 = min(START_FREQ_HZ, ELEC_FREQ_HZ);
  float d0 = min(START_DUTY_PERCENT, DUTY_PERCENT);
  float f  = f0 + (ELEC_FREQ_HZ - f0) * k;
  float d  = d0 + (DUTY_PERCENT - d0) * k;
  d = min(d, MAX_DUTY_PERCENT);

  stepPeriodUs = (uint32_t)(1000000.0f / (6.0f * f));   // 6 steps per electrical cycle
  uint32_t D = (uint32_t)(d * pwmPeriod / 100.0f);
  if (D != curD) { curD = D; needApply = true; }
}

// ---- Enable / disable ----
void setEnable(bool on) {
  if (on) {
    allOff();
    for (uint8_t p = 0; p < 3; p++) setCh(2 * p + 1, CH_ON);   // charge bootstrap caps
    delay(BOOT_CHARGE_MS);
    allOff();
    stepIdx    = 0;
    lastStepUs = micros();
    runStartMs = millis();
    curD       = 0;
    updateRamp(runStartMs);
    lastRampMs = runStartMs;
    needApply  = true;
    enabled    = true;
  } else {
    enabled = false;
    allOff();
  }
}

// Reconfigure timer safely (all outputs off while doing so)
void reconfigPwm() {
  allOff();
  if (!timerConfig()) Serial.println("ERR: ledc_timer_config failed");
  curD = 0;
  if (enabled) { updateRamp(millis()); lastRampMs = millis(); needApply = true; }
}

// ---- Status / serial ----
void printStatus() {
  float realDt = deadTicks * 1e9f / ((float)PWM_FREQ_HZ * (float)pwmPeriod);
  Serial.printf("en=%d dir=%d duty=%.1f%% max=%.1f%% f=%.1fHz pwm=%luHz(%ubit) dt=%luns(real %.0fns,%lu ticks) ramp=%lums\n",
                enabled, DIRECTION, DUTY_PERCENT, MAX_DUTY_PERCENT, ELEC_FREQ_HZ,
                (unsigned long)PWM_FREQ_HZ, pwmBits, (unsigned long)DEAD_TIME_NS,
                realDt, (unsigned long)deadTicks, (unsigned long)RAMP_TIME_MS);
}

void handleCommand(const char *s) {
  char  c = tolower(s[0]);
  float v = atof(s + 1);
  switch (c) {
    case 'd': DUTY_PERCENT     = constrain(v, 0.0f, 100.0f); break;
    case 'm': MAX_DUTY_PERCENT = constrain(v, 10.0f, 98.0f); break;
    case 't': DEAD_TIME_NS     = (uint32_t)constrain(v, 0.0f, 5000.0f); reconfigPwm(); break;
    case 'f': ELEC_FREQ_HZ     = constrain(v, 1.0f, 500.0f); break;
    case 'p': PWM_FREQ_HZ      = (uint32_t)constrain(v, 1000.0f, 40000.0f); reconfigPwm(); break;
    case 'r': RAMP_TIME_MS     = (uint32_t)max(v, 0.0f); runStartMs = millis(); break;
    case 'x': DIRECTION        = (v > 0.5f) ? 1 : 0; break;
    case 'e': setEnable(v > 0.5f); break;
    case '?': break;
    default:  Serial.println("cmds: d m t f p r x e ?"); return;
  }
  printStatus();
}

void readSerial() {
  static char    buf[24];
  static uint8_t n = 0;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (n) { buf[n] = 0; handleCommand(buf); n = 0; }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = ch;
    }
  }
}

// ---- Arduino ----
void setup() {
  Serial.begin(115200);
  if (!timerConfig()) Serial.println("ERR: ledc_timer_config failed");

  for (uint8_t i = 0; i < 6; i++) {
    ledc_channel_config_t c = {};
    c.gpio_num   = GATE_PIN[i];
    c.speed_mode = LEDC_MODE;
    c.channel    = (ledc_channel_t)i;
    c.intr_type  = LEDC_INTR_DISABLE;
    c.timer_sel  = LEDC_TMR;
    c.duty       = 0;
    c.hpoint     = 0;
    ledc_channel_config(&c);
    cache[i].valid = false;
  }
  allOff();
  setEnable(ENABLE_ON_BOOT);
  Serial.println("BLDC open-loop full-bridge ready. Send ? for status, e1 to run.");
  printStatus();
}

void loop() {
  readSerial();
  if (!enabled) return;

  uint32_t nowMs = millis();
  if (nowMs != lastRampMs) { lastRampMs = nowMs; updateRamp(nowMs); }

  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastStepUs) >= stepPeriodUs) {
    lastStepUs += stepPeriodUs;
    stepIdx = (DIRECTION == 0) ? (stepIdx + 1) % 6 : (stepIdx + 5) % 6;
    needApply = true;
  }

  if (needApply) {
    needApply = false;
    applyOutput(stepIdx, curD);
  }
}
