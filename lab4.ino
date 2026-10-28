#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// ---------- Hardware config ----------
#define I2C_SDA        21
#define I2C_SCL        22
#define OLED_ADDR      0x3C
#define MPU_ADDR       0x68
#define SCREEN_W       128
#define SCREEN_H       64
#define RESET_BTN_PIN  0      // BOOT button

// ---------- Touch sensors ----------
#define YELLOW_CAP 12
#define WHITE_CAP  13
#define BLACK_CAP  14

// ---------- Gyro axis mapping ----------
#define SIGN_X  1.0f
#define SIGN_Y  -1.0f
#define SIGN_Z  1.0f

// Which raw sensor axis (0=X, 1=Y, 2=Z) drives each cube rotation
#define MAP_X  0
#define MAP_Y  2
#define MAP_Z  1

// ---------- Behavior tuning ----------
#define SPEED_MIN      0.25f   // rotation speed multiplier limits
#define SPEED_MAX      4.0f
#define SPEED_RATE     1.0f    // doublings per second while touching
#define FISHEYE_RATE   0.4f    // fisheye level change per second (0..1)
#define FISHEYE_START  0.85f   // 0 = mild, 1 = extreme
#define CAM_FAR        6.0f    // camera distance at fisheye level 0
#define CAM_NEAR       1.9f    // camera distance at fisheye level 1 (must be > 1.74)
#define TARGET_RADIUS  24.0f   // approx. on-screen radius of the cube in pixels
#define EDGE_SEGS      10
#define GYRO_DEADZONE  0.01f   // rad/s

// ---------- Reset gesture (touch all three sensors) ----------
#define RESET_HOLD_MS  1500    // how long to hold all three sensors

// ---------- Touch detection tuning ----------
#define ON_FRAC    0.50f   // activate when 50% of the way from untouched to touched
#define OFF_FRAC   0.35f   // release when it falls back below 35% (hysteresis)
#define MIN_DIFF   5.0f    // minimum touched/untouched difference for valid calibration

enum TouchAction { ACT_NONE, ACT_SPEED_UP, ACT_SPEED_DOWN, ACT_FISH_UP, ACT_FISH_DOWN };

Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

// ---------- State ----------
float R[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
float gyroBias[3] = {0, 0, 0};
unsigned long lastMicros = 0;
uint16_t frameCount = 0;

float speedGain = 1.0f;
float fisheyeLevel = FISHEYE_START;
float camDist = 2.4f;
float fishK = 30.0f;

enum { YEL = 0, WHT = 1, BLK = 2 };
const uint8_t  CAP_PINS[3]  = { YELLOW_CAP, WHITE_CAP, BLACK_CAP };
const char*    CAP_NAMES[3] = { "YELLOW", "WHITE", "BLACK" };
float touchedAvg[3]   = {0, 0, 0};
float untouchedAvg[3] = {0, 0, 0};
bool  capActive[3]    = {false, false, false};

const int8_t VERTS[8][3] = {
  {-1,-1,-1},{ 1,-1,-1},{-1, 1,-1},{ 1, 1,-1},
  {-1,-1, 1},{ 1,-1, 1},{-1, 1, 1},{ 1, 1, 1}
};

// =====================================================================
//  Gyro (raw register access: works with MPU6050/6500 and clones)
// =====================================================================
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void mpuInit() {
  mpuWrite(0x6B, 0x01);   // wake up, gyro X PLL clock
  delay(100);
  mpuWrite(0x1B, 0x08);   // +/-500 deg/s
  mpuWrite(0x1A, 0x03);   // low-pass filter ~44 Hz
}

bool readRawGyro(float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6) != 6) return false;

  int16_t x = (Wire.read() << 8) | Wire.read();
  int16_t y = (Wire.read() << 8) | Wire.read();
  int16_t z = (Wire.read() << 8) | Wire.read();

  const float scale = (1.0f / 65.5f) * (PI / 180.0f);
  gx = x * scale; gy = y * scale; gz = z * scale;
  return true;
}

void readGyro(float &gx, float &gy, float &gz) {
  float x, y, z;
  if (!readRawGyro(x, y, z)) { gx = gy = gz = 0; return; }

  float raw[3] = { x - gyroBias[0], y - gyroBias[1], z - gyroBias[2] };
  gx = SIGN_X * raw[MAP_X];
  gy = SIGN_Y * raw[MAP_Y];
  gz = SIGN_Z * raw[MAP_Z];

  if (fabsf(gx) < GYRO_DEADZONE) gx = 0;
  if (fabsf(gy) < GYRO_DEADZONE) gy = 0;
  if (fabsf(gz) < GYRO_DEADZONE) gz = 0;
}

void calibrateGyro() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 20);
  display.println("Calibrating gyro...");
  display.println("Keep it still!");
  display.display();

  const int N = 300;
  float sum[3] = {0, 0, 0};
  int good = 0;
  for (int i = 0; i < N; i++) {
    float x, y, z;
    if (readRawGyro(x, y, z)) { sum[0] += x; sum[1] += y; sum[2] += z; good++; }
    delay(5);
  }
  if (good > 0) for (int i = 0; i < 3; i++) gyroBias[i] = sum[i] / good;
}

// =====================================================================
//  Touch calibration and reading
// =====================================================================
float averageTouch(uint8_t pin, uint32_t ms) {
  uint32_t start = millis();
  double sum = 0;
  int n = 0;
  while (millis() - start < ms) {
    sum += touchRead(pin);
    n++;
    delay(5);
  }
  return n ? (float)(sum / n) : 0;
}

void drawCalScreen(const char* header, const char* name, int countdown, bool hold) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(header);
  display.setTextSize(2);
  display.setCursor(0, 12);
  display.print(name);
  display.setTextSize(3);
  display.setCursor(0, 38);
  if (hold) display.print("HOLD");
  else      display.print(countdown);
  display.display();
}

// 3 second countdown, then a 1 second average. Returns the average.
float calibrationStep(int stepNum, bool touching, int idx) {
  char header[32];
  snprintf(header, sizeof(header), "%s (%d/6)", touching ? "Touch" : "Do NOT touch", stepNum);

  for (int s = 3; s >= 1; s--) {
    drawCalScreen(header, CAP_NAMES[idx], s, false);
    delay(1000);
  }
  drawCalScreen(header, CAP_NAMES[idx], 0, true);
  return averageTouch(CAP_PINS[idx], 1000);
}

bool calibrateTouch() {
  // Phase 1: touch each sensor
  for (int i = 0; i < 3; i++) touchedAvg[i] = calibrationStep(i + 1, true, i);
  // Phase 2: do not touch each sensor
  for (int i = 0; i < 3; i++) untouchedAvg[i] = calibrationStep(i + 4, false, i);

  bool ok = true;
  Serial.println("--- Touch calibration ---");
  for (int i = 0; i < 3; i++) {
    float diff = touchedAvg[i] - untouchedAvg[i];
    float thrOn  = untouchedAvg[i] + ON_FRAC  * diff;
    float thrOff = untouchedAvg[i] + OFF_FRAC * diff;
    Serial.printf("%-6s untouched=%.1f touched=%.1f  on-threshold=%.1f off-threshold=%.1f\n",
                  CAP_NAMES[i], untouchedAvg[i], touchedAvg[i], thrOn, thrOff);

    if (fabsf(diff) < MIN_DIFF) {
      ok = false;
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, 10);
      display.println("Weak signal on");
      display.setTextSize(2);
      display.println(CAP_NAMES[i]);
      display.setTextSize(1);
      display.println("Check wiring, retry");
      display.display();
      delay(2500);
    }
  }
  return ok;
}

// Smoothed reading for one sensor
float readTouchAvg(uint8_t pin) {
  float sum = 0;
  for (int i = 0; i < 3; i++) sum += touchRead(pin);
  return sum / 3.0f;
}

// Updates capActive[] using the calibration (works whether the value rises or falls on touch)
void updateTouch() {
  for (int i = 0; i < 3; i++) {
    float diff = touchedAvg[i] - untouchedAvg[i];
    if (fabsf(diff) < 0.001f) { capActive[i] = false; continue; }
    float score = (readTouchAvg(CAP_PINS[i]) - untouchedAvg[i]) / diff;  // 0 = untouched, 1 = touched
    if (capActive[i]) { if (score < OFF_FRAC) capActive[i] = false; }
    else              { if (score > ON_FRAC)  capActive[i] = true;  }
  }
}

const char* actionName(TouchAction a) {
  switch (a) {
    case ACT_SPEED_UP:   return "SPEED INCREASING   (yellow)";
    case ACT_SPEED_DOWN: return "SPEED DECREASING   (yellow + white)";
    case ACT_FISH_UP:    return "FISHEYE INCREASING (black)";
    case ACT_FISH_DOWN:  return "FISHEYE DECREASING (black + white)";
    default:             return "none";
  }
}

// Yellow = speed up, Yellow+White = slow down
// Black = more fisheye, Black+White = less fisheye
void applyTouchControls(float dt) {
  static TouchAction lastAction = ACT_NONE;
  static unsigned long lastPrintMs = 0;

  bool y = capActive[YEL], w = capActive[WHT], b = capActive[BLK];

  // Decide which action (if any) is active
  TouchAction action = ACT_NONE;
  if (y && !b)      action = w ? ACT_SPEED_DOWN : ACT_SPEED_UP;
  else if (b && !y) action = w ? ACT_FISH_DOWN  : ACT_FISH_UP;
  // White alone, or yellow+black together, does nothing.

  // Apply the action
  switch (action) {
    case ACT_SPEED_UP:
    case ACT_SPEED_DOWN: {
      float dir = (action == ACT_SPEED_UP) ? 1.0f : -1.0f;
      speedGain *= powf(2.0f, dir * SPEED_RATE * dt);
      speedGain = constrain(speedGain, SPEED_MIN, SPEED_MAX);
      break;
    }
    case ACT_FISH_UP:
    case ACT_FISH_DOWN: {
      float dir = (action == ACT_FISH_UP) ? 1.0f : -1.0f;
      fisheyeLevel += dir * FISHEYE_RATE * dt;
      fisheyeLevel = constrain(fisheyeLevel, 0.0f, 1.0f);
      break;
    }
    default:
      break;
  }

  // Serial reporting
  unsigned long nowMs = millis();

  if (action != lastAction) {
    if (action == ACT_NONE) {
      Serial.printf("Released (last: %s) -> speed x%.2f, fisheye %d%%\n",
                    actionName(lastAction), speedGain, (int)(fisheyeLevel * 100.0f + 0.5f));
    } else {
      Serial.printf("Action: %s\n", actionName(action));
      lastPrintMs = 0;   // print a value update immediately
    }
    lastAction = action;
  }

  if (action != ACT_NONE && nowMs - lastPrintMs >= 250) {
    lastPrintMs = nowMs;
    if (action == ACT_SPEED_UP || action == ACT_SPEED_DOWN) {
      const char* limit = "";
      if (speedGain >= SPEED_MAX)      limit = "  [MAX]";
      else if (speedGain <= SPEED_MIN) limit = "  [MIN]";
      Serial.printf("  speed = x%.2f%s\n", speedGain, limit);
    } else {
      const char* limit = "";
      if (fisheyeLevel >= 1.0f)      limit = "  [MAX]";
      else if (fisheyeLevel <= 0.0f) limit = "  [MIN]";
      Serial.printf("  fisheye = %d%%  (camera distance %.2f)%s\n",
                    (int)(fisheyeLevel * 100.0f + 0.5f), camDist, limit);
    }
  }
}

// =====================================================================
//  Rotation and rendering
// =====================================================================
void resetOrientation() {
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      R[i][j] = (i == j) ? 1.0f : 0.0f;
}

// Hold all three touch sensors for RESET_HOLD_MS to reset the cube
// orientation, speed and fisheye. Fires once per hold.
void checkResetGesture() {
  static unsigned long heldSince = 0;
  static bool fired = false;

  bool all = capActive[YEL] && capActive[WHT] && capActive[BLK];

  if (!all) {              // released (or never held): re-arm
    heldSince = 0;
    fired = false;
    return;
  }

  if (fired) return;       // already reset during this hold

  unsigned long now = millis();
  if (heldSince == 0) {
    heldSince = now;
    Serial.println("Reset gesture: keep holding all three...");
  }

  if (now - heldSince >= RESET_HOLD_MS) {
    resetOrientation();
    speedGain = 1.0f;
    fisheyeLevel = FISHEYE_START;
    fired = true;
    Serial.println("RESET: cube position, speed and fisheye restored");
  }
}

void integrateRotation(float wx, float wy, float wz, float dt) {
  float x = wx * dt, y = wy * dt, z = wz * dt;
  float ang = sqrtf(x*x + y*y + z*z);
  if (ang < 1e-7f) return;

  float kx = x/ang, ky = y/ang, kz = z/ang;
  float c = cosf(ang), s = sinf(ang), t = 1.0f - c;

  float dR[3][3] = {
    { t*kx*kx + c,     t*kx*ky - s*kz,  t*kx*kz + s*ky },
    { t*kx*ky + s*kz,  t*ky*ky + c,     t*ky*kz - s*kx },
    { t*kx*kz - s*ky,  t*ky*kz + s*kx,  t*kz*kz + c    }
  };

  float N[3][3];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      N[i][j] = R[i][0]*dR[0][j] + R[i][1]*dR[1][j] + R[i][2]*dR[2][j];
  memcpy(R, N, sizeof(R));
}

void orthonormalize() {
  float n = sqrtf(R[0][0]*R[0][0] + R[0][1]*R[0][1] + R[0][2]*R[0][2]);
  for (int j = 0; j < 3; j++) R[0][j] /= n;

  float d = R[0][0]*R[1][0] + R[0][1]*R[1][1] + R[0][2]*R[1][2];
  for (int j = 0; j < 3; j++) R[1][j] -= d * R[0][j];
  n = sqrtf(R[1][0]*R[1][0] + R[1][1]*R[1][1] + R[1][2]*R[1][2]);
  for (int j = 0; j < 3; j++) R[1][j] /= n;

  R[2][0] = R[0][1]*R[1][2] - R[0][2]*R[1][1];
  R[2][1] = R[0][2]*R[1][0] - R[0][0]*R[1][2];
  R[2][2] = R[0][0]*R[1][1] - R[0][1]*R[1][0];
}

void rotatePoint(const float v[3], float out[3]) {
  for (int i = 0; i < 3; i++)
    out[i] = R[i][0]*v[0] + R[i][1]*v[1] + R[i][2]*v[2];
}

// Fisheye strength sets how close the camera is. K is rescaled so the cube
// stays roughly the same size on screen at every level.
void updateCamera() {
  camDist = CAM_FAR + (CAM_NEAR - CAM_FAR) * fisheyeLevel;
  fishK = TARGET_RADIUS / atan2f(1.41f, camDist - 1.0f);
}

bool fisheyeProject(const float p[3], int &sx, int &sy) {
  float x = p[0], y = p[1], z = p[2] + camDist;
  if (z < 0.05f) return false;

  float r = sqrtf(x*x + y*y);
  float theta = atan2f(r, z);
  float scale = (r > 1e-6f) ? (fishK * theta / r) : fishK / z;

  sx = (int)lroundf(SCREEN_W / 2 + x * scale);
  sy = (int)lroundf(SCREEN_H / 2 + y * scale);
  return true;
}

void drawCube() {
  for (int i = 0; i < 8; i++) {
    for (int b = 0; b < 3; b++) {
      int j = i ^ (1 << b);
      if (j < i) continue;

      int px = 0, py = 0;
      bool havePrev = false;

      for (int s = 0; s <= EDGE_SEGS; s++) {
        float u = (float)s / EDGE_SEGS;
        float v[3], w[3];
        for (int k = 0; k < 3; k++)
          v[k] = VERTS[i][k] + (VERTS[j][k] - VERTS[i][k]) * u;
        rotatePoint(v, w);

        int sx, sy;
        if (fisheyeProject(w, sx, sy)) {
          if (havePrev) display.drawLine(px, py, sx, sy, SSD1306_WHITE);
          px = sx; py = sy; havePrev = true;
        } else {
          havePrev = false;
        }
      }
    }
  }
}

// Small status readout in the top corners: speed (left), fisheye % (right)
void drawHud() {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("x");
  display.print(speedGain, 1);

  char buf[8];
  snprintf(buf, sizeof(buf), "F%d", (int)(fisheyeLevel * 100.0f + 0.5f));
  display.setCursor(SCREEN_W - 6 * (int)strlen(buf), 0);
  display.print(buf);
}

// =====================================================================
void setup() {
  delay(5000);
  Serial.begin(115200);
  pinMode(RESET_BTN_PIN, INPUT_PULLUP);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 not found - check wiring/address");
    while (true) delay(1000);
  }

  mpuInit();

  // Repeat until all three sensors give a usable signal
  while (!calibrateTouch()) { }

  calibrateGyro();
  lastMicros = micros();
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;

  if (digitalRead(RESET_BTN_PIN) == LOW) resetOrientation();

  updateTouch();
  checkResetGesture();
  applyTouchControls(dt);
  updateCamera();

  float gx, gy, gz;
  readGyro(gx, gy, gz);
  integrateRotation(gx * speedGain, gy * speedGain, gz * speedGain, dt);

  if (++frameCount >= 20) {
    frameCount = 0;
    orthonormalize();
  }

  display.clearDisplay();
  drawCube();
  drawHud();
  display.display();
}