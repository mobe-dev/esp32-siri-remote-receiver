#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include <BLEDevice.h>
#include <BLERemoteCharacteristic.h>
#include <BLERemoteService.h>
#include <BLESecurity.h>
#include <USB.h>
#include <USBHID.h>
#include <USBHIDConsumerControl.h>
#include <USBHIDKeyboard.h>
#include <cstdarg>
#include <cstring>
#include <strings.h>

#if ARDUINO_USB_MODE
#error This firmware expects native USB device mode on the ESP32-S3.
#endif

#ifndef SIRI_REMOTE_MAC
#define SIRI_REMOTE_MAC ""
#endif

#ifndef SIRI_TOUCH_SENSITIVITY
#define SIRI_TOUCH_SENSITIVITY 8
#endif

#ifndef SIRI_EDGE_FILTER
#define SIRI_EDGE_FILTER 0
#endif

#ifndef SIRI_SWAP_MEDIA_NEXT_PREV
#define SIRI_SWAP_MEDIA_NEXT_PREV 0
#endif

#ifndef SIRI_PRECISION_TOUCHPAD
#define SIRI_PRECISION_TOUCHPAD 0
#endif

#if SIRI_PRECISION_TOUCHPAD
#error "SIRI_PRECISION_TOUCHPAD cannot be enabled on the current Siri Remote parser: Windows Precision Touchpad requires at least 3 concurrent contacts, but this firmware only decodes up to 2-contact packets."
#endif

namespace {

constexpr int kNeoPixelPin = 48;
constexpr uint16_t kNumLeds = 1;
constexpr uint8_t kBrightness = 32;

constexpr uint16_t kBatteryServiceUuid = 0x180F;
constexpr uint16_t kHidServiceUuid = 0x1812;
constexpr uint16_t kBatteryLevelUuid = 0x2A19;
constexpr uint16_t kPowerStateUuid = 0x2A1A;

constexpr uint16_t kInputReportHandle = 0x0023;
constexpr uint16_t kEnableInputHandle = 0x001D;

constexpr uint8_t kMagicEnableInput = 0xAF;
constexpr uint8_t kTouchEventMarker = 50;

constexpr uint8_t kPowerCharging = 171;
constexpr uint8_t kPowerDischarging = 175;
constexpr uint8_t kPowerPluggedIn = 187;

constexpr uint8_t kButtonReleased = 0x00;
constexpr uint8_t kButtonAirplay = 0x01;
constexpr uint8_t kButtonVolumeUp = 0x02;
constexpr uint8_t kButtonVolumeDown = 0x04;
constexpr uint8_t kButtonPlayPause = 0x08;
constexpr uint8_t kButtonSiri = 0x10;
constexpr uint8_t kButtonMenu = 0x20;
constexpr uint8_t kButtonTouchpad2 = 0x40;
constexpr uint8_t kButtonTouchpad = 0x80;
constexpr uint8_t MOUSE_LEFT = 0x01;
constexpr uint8_t MOUSE_RIGHT = 0x02;

constexpr uint32_t kReconnectDelayMs = 2000;
constexpr uint16_t kScanIntervalMs = 30;
constexpr uint16_t kScanWindowMs = 30;
constexpr uint32_t kStatusPrintIntervalMs = 4000;
constexpr uint32_t kTouchLogIntervalMs = 75;
constexpr uint32_t kScrollReleaseGateMs = 120;
constexpr uint32_t kTapMaxDurationMs = 250;
constexpr uint32_t kDoubleTapWindowMs = 350;
constexpr uint32_t kConsumerPulseMs = 35;
constexpr size_t kDebugBufferSize = 4096;
constexpr uint8_t kMinTouchPressure = 8;
constexpr int kScrollSensitivity = 1;
constexpr int kScrollSmoothingNumerator = 4;
constexpr int kScrollSmoothingDenominator = 8;
constexpr int kScrollResponseScale = 4;
constexpr int kScrollBaseGain = 5;
constexpr int kScrollAccelerationThreshold = 2;
constexpr int kScrollAccelerationBoost = 3;
constexpr int kPointerSmoothingNumerator = 5;
constexpr int kPointerSmoothingDenominator = 8;
constexpr int kPointerResponseScale = 8;
constexpr int kPointerBaseGain = 3;
constexpr int kPointerAccelerationThreshold = 14;
constexpr int kPointerAccelerationDivisor = 2;
constexpr int kTouchEdgeMarginX = 2;
constexpr int kTouchEdgeMarginY = 3;
constexpr int kTouchMaxX = 108;
constexpr int kTouchMaxY = 82;
constexpr uint8_t kDefaultWheelResolutionMultiplier = 7;

class SmoothScrollHIDMouse : public USBHIDDevice {
 public:
  SmoothScrollHIDMouse() : hid(), buttons_(0), wheelResolutionMultiplier_(kDefaultWheelResolutionMultiplier) {
    static bool initialized = false;
    if (!initialized) {
      initialized = true;
      hid.addDevice(this, sizeof(reportDescriptor));
    }
  }

  void begin() {
    hid.begin();
  }

  void click(uint8_t button = MOUSE_LEFT) {
    buttons_ = button;
    move(0, 0);
    buttons_ = 0;
    move(0, 0);
  }

  void move(int8_t x, int8_t y, int8_t wheel = 0, int8_t pan = 0) {
    hid_mouse_report_t report = {
        .buttons = buttons_,
        .x = x,
        .y = y,
        .wheel = wheel,
        .pan = pan,
    };
    hid.SendReport(HID_REPORT_ID_MOUSE, &report, sizeof(report));
  }

  void press(uint8_t button = MOUSE_LEFT) {
    setButtons(buttons_ | button);
  }

  void release(uint8_t button = MOUSE_LEFT) {
    setButtons(buttons_ & ~button);
  }

  bool isPressed(uint8_t button = MOUSE_LEFT) const {
    return (buttons_ & button) != 0;
  }

  uint16_t _onGetDescriptor(uint8_t *buffer) override {
    memcpy(buffer, reportDescriptor, sizeof(reportDescriptor));
    return sizeof(reportDescriptor);
  }

  uint16_t _onGetFeature(uint8_t report_id, uint8_t *buffer, uint16_t len) override {
    if (report_id != HID_REPORT_ID_MOUSE || len == 0) {
      return 0;
    }
    buffer[0] = wheelResolutionMultiplier_;
    return 1;
  }

  void _onSetFeature(uint8_t report_id, const uint8_t *buffer, uint16_t len) override {
    if (report_id != HID_REPORT_ID_MOUSE || len == 0) {
      return;
    }
    wheelResolutionMultiplier_ = buffer[0] > 15 ? 15 : buffer[0];
  }

 private:
  void setButtons(uint8_t buttons) {
    if (buttons_ == buttons) {
      return;
    }
    buttons_ = buttons;
    move(0, 0);
  }

  USBHID hid;
  uint8_t buttons_;
  uint8_t wheelResolutionMultiplier_;

  static constexpr uint8_t reportDescriptor[] = {
      0x05, 0x01,                    // Usage Page (Generic Desktop)
      0x09, 0x02,                    // Usage (Mouse)
      0xA1, 0x01,                    // Collection (Application)
      0x85, HID_REPORT_ID_MOUSE,     //   Report ID
      0x09, 0x01,                    //   Usage (Pointer)
      0xA1, 0x00,                    //   Collection (Physical)
      0x05, 0x09,                    //     Usage Page (Button)
      0x19, 0x01,                    //     Usage Minimum (1)
      0x29, 0x05,                    //     Usage Maximum (5)
      0x15, 0x00,                    //     Logical Minimum (0)
      0x25, 0x01,                    //     Logical Maximum (1)
      0x95, 0x05,                    //     Report Count (5)
      0x75, 0x01,                    //     Report Size (1)
      0x81, 0x02,                    //     Input (Data,Var,Abs)
      0x95, 0x01,                    //     Report Count (1)
      0x75, 0x03,                    //     Report Size (3)
      0x81, 0x01,                    //     Input (Const,Array,Abs)
      0x05, 0x01,                    //     Usage Page (Generic Desktop)
      0x09, 0x30,                    //     Usage (X)
      0x09, 0x31,                    //     Usage (Y)
      0x15, 0x81,                    //     Logical Minimum (-127)
      0x25, 0x7F,                    //     Logical Maximum (127)
      0x75, 0x08,                    //     Report Size (8)
      0x95, 0x02,                    //     Report Count (2)
      0x81, 0x06,                    //     Input (Data,Var,Rel)
      0xA1, 0x02,                    //     Collection (Logical)
      0x09, 0x48,                    //       Usage (Resolution Multiplier)
      0x15, 0x00,                    //       Logical Minimum (0)
      0x25, 0x0F,                    //       Logical Maximum (15)
      0x35, 0x01,                    //       Physical Minimum (1)
      0x45, 0x10,                    //       Physical Maximum (16)
      0x75, 0x08,                    //       Report Size (8)
      0x95, 0x01,                    //       Report Count (1)
      0xB1, 0x02,                    //       Feature (Data,Var,Abs)
      0x09, 0x38,                    //       Usage (Wheel)
      0x15, 0x81,                    //       Logical Minimum (-127)
      0x25, 0x7F,                    //       Logical Maximum (127)
      0x75, 0x08,                    //       Report Size (8)
      0x95, 0x01,                    //       Report Count (1)
      0x81, 0x06,                    //       Input (Data,Var,Rel)
      0x05, 0x0C,                    //       Usage Page (Consumer)
      0x0A, 0x38, 0x02,              //       Usage (AC Pan)
      0x15, 0x81,                    //       Logical Minimum (-127)
      0x25, 0x7F,                    //       Logical Maximum (127)
      0x75, 0x08,                    //       Report Size (8)
      0x95, 0x01,                    //       Report Count (1)
      0x81, 0x06,                    //       Input (Data,Var,Rel)
      0xC0,                          //     End Collection
      0xC0,                          //   End Collection
      0xC0                           // End Collection
  };
};

constexpr uint8_t SmoothScrollHIDMouse::reportDescriptor[];

Adafruit_NeoPixel strip(kNumLeds, kNeoPixelPin, NEO_GRB + NEO_KHZ800);
USBHID HID;
SmoothScrollHIDMouse Mouse;
USBHIDConsumerControl ConsumerControl;
USBHIDKeyboard Keyboard;

BLEClient *bleClient = nullptr;
BLEAdvertisedDevice *remoteDevice = nullptr;
BLERemoteCharacteristic *batteryCharacteristic = nullptr;
BLERemoteCharacteristic *powerCharacteristic = nullptr;
BLERemoteCharacteristic *inputCharacteristic = nullptr;
BLERemoteCharacteristic *enableInputCharacteristic = nullptr;

volatile bool bleConnected = false;
volatile bool scanRunning = false;
volatile bool shouldConnect = false;
volatile bool bleSessionDirty = false;
volatile bool rescanRequested = false;
bool usbReadyLogged = false;
bool profileSwitchSuppressUntilRelease = false;
bool profileSelectionActive = false;
bool profileSelectionModifierUsed = false;
uint8_t lastButtonMask = 0;
uint8_t lastRawButtonMask = 0;
uint16_t lastConsumerUsage = 0;
uint32_t consumerPulseReleaseAtMs = 0;
volatile uint32_t reconnectBlockedUntilMs = 0;
uint32_t lastStatusPrintMs = 0;
uint32_t lastTouchLogMs = 0;
volatile uint32_t lastDisconnectMs = 0;
volatile uint32_t lastScanStartedMs = 0;
volatile uint32_t lastRemoteAdvertisementMs = 0;
volatile uint32_t lastConnectStartedMs = 0;
uint32_t scrollReleaseGateUntilMs = 0;
int16_t previousTouchX = INT16_MIN;
int16_t previousTouchY = INT16_MIN;
uint8_t previousTouchCount = 0;
int filteredPointerDx = 0;
int filteredPointerDy = 0;
int pointerRemainderX = 0;
int pointerRemainderY = 0;
int filteredScrollWheel = 0;
int filteredScrollPan = 0;
int scrollRemainderWheel = 0;
int scrollRemainderPan = 0;
bool tapCandidateActive = false;
bool tapCandidateMoved = false;
bool touchDragActive = false;
uint32_t tapCandidateStartMs = 0;
uint32_t lastTapReleaseMs = 0;
int16_t tapCandidateStartX = INT16_MIN;
int16_t tapCandidateStartY = INT16_MIN;
int16_t lastTapReleaseX = INT16_MIN;
int16_t lastTapReleaseY = INT16_MIN;
String debugBuffer;

const char *kRemoteMac = SIRI_REMOTE_MAC;

enum class InputProfile : uint8_t {
  Mouse,
  PowerPoint,
  Media,
};

enum class LedState : uint8_t {
  Idle,
  Scanning,
  Connecting,
  Connected,
  Activity,
  Error,
};

LedState ledState = LedState::Idle;
uint32_t ledStateUntilMs = 0;
InputProfile currentProfile = InputProfile::Mouse;

bool hidReady();
void appendDebugBuffer(const char *message);
void debugLogf(const char *fmt, ...);
void resetTouchTracking();
void updateConsumerPulse();
void triggerPlayAction();
void resetBleSession();
void invalidateRemoteCharacteristics();

bool haveRemoteCharacteristicCache() {
  return batteryCharacteristic != nullptr && powerCharacteristic != nullptr &&
         inputCharacteristic != nullptr && enableInputCharacteristic != nullptr;
}

void invalidateRemoteCharacteristics() {
  batteryCharacteristic = nullptr;
  powerCharacteristic = nullptr;
  inputCharacteristic = nullptr;
  enableInputCharacteristic = nullptr;
}

void markBleSessionDirty(const char *reason) {
  if (!bleSessionDirty) {
    debugLogf("Marking BLE session dirty: %s\n", reason);
  }
  bleSessionDirty = true;
}

void logReconnectPhase(const char *phase, uint32_t timestampMs) {
  if (lastDisconnectMs == 0) {
    debugLogf("Reconnect phase %s at %lu ms\n", phase, static_cast<unsigned long>(timestampMs));
    return;
  }

  debugLogf("Reconnect phase %s at %lu ms (+%lu ms since disconnect)\n",
            phase,
            static_cast<unsigned long>(timestampMs),
            static_cast<unsigned long>(timestampMs - lastDisconnectMs));
}

bool hasRemoteMac() {
  return kRemoteMac[0] != '\0' && strcmp(kRemoteMac, "48:A9:1C:00:00:00") != 0;
}

const char *profileName(InputProfile profile) {
  switch (profile) {
    case InputProfile::Mouse:
      return "mouse";
    case InputProfile::PowerPoint:
      return "powerpoint";
    case InputProfile::Media:
      return "media";
  }
  return "unknown";
}

void profileLedColor(InputProfile profile, uint8_t &red, uint8_t &green, uint8_t &blue) {
  switch (profile) {
    case InputProfile::Mouse:
      red = 0;
      green = 18;
      blue = 0;
      break;
    case InputProfile::PowerPoint:
      red = 18;
      green = 10;
      blue = 0;
      break;
    case InputProfile::Media:
      red = 0;
      green = 10;
      blue = 18;
      break;
  }
}

void setLedColor(uint8_t red, uint8_t green, uint8_t blue) {
  strip.setPixelColor(0, strip.Color(red, green, blue));
  strip.show();
}

void appendDebugBuffer(const char *message) {
  debugBuffer += message;
  if (debugBuffer.length() > kDebugBufferSize) {
    debugBuffer.remove(0, debugBuffer.length() - kDebugBufferSize);
  }
}

void debugPrintRaw(const char *message) {
  Serial.print(message);
  appendDebugBuffer(message);
}

void debugLogf(const char *fmt, ...) {
  char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  debugPrintRaw(buffer);
}

void debugLogln(const char *message) {
  debugLogf("%s\n", message);
}

bool shouldLogTouchNow() {
  const uint32_t now = millis();
  if (now - lastTouchLogMs < kTouchLogIntervalMs) {
    return false;
  }
  lastTouchLogMs = now;
  return true;
}

void setLedState(LedState state, uint32_t durationMs = 0) {
  ledState = state;
  ledStateUntilMs = durationMs == 0 ? 0 : millis() + durationMs;

  uint8_t profileRed = 0;
  uint8_t profileGreen = 0;
  uint8_t profileBlue = 0;
  profileLedColor(currentProfile, profileRed, profileGreen, profileBlue);

  switch (state) {
    case LedState::Idle:
      setLedColor(profileRed / 3, profileGreen / 3, profileBlue / 3);
      break;
    case LedState::Scanning:
      setLedColor(10, 6, 0);
      break;
    case LedState::Connecting:
      setLedColor(0, 0, 24);
      break;
    case LedState::Connected:
      setLedColor(profileRed, profileGreen, profileBlue);
      break;
    case LedState::Activity:
      setLedColor(static_cast<uint8_t>(profileRed + 8 > 24 ? 24 : profileRed + 8),
                  static_cast<uint8_t>(profileGreen + 8 > 24 ? 24 : profileGreen + 8),
                  static_cast<uint8_t>(profileBlue + 8 > 24 ? 24 : profileBlue + 8));
      break;
    case LedState::Error:
      setLedColor(24, 0, 0);
      break;
  }
}

void updateLedState() {
  if (ledState == LedState::Activity && ledStateUntilMs != 0 && millis() > ledStateUntilMs) {
    setLedState(bleConnected ? LedState::Connected : LedState::Scanning);
  }
}

int8_t clampMouseDelta(int value) {
  if (value < -127) {
    return -127;
  }
  if (value > 127) {
    return 127;
  }
  return static_cast<int8_t>(value);
}

bool hidReady() {
  return HID.ready();
}

void releaseMouseButtons() {
  Mouse.release(MOUSE_LEFT);
  Mouse.release(MOUSE_RIGHT);
}

void releaseConsumerControl() {
  if (lastConsumerUsage != 0) {
    ConsumerControl.release();
    lastConsumerUsage = 0;
  }
  consumerPulseReleaseAtMs = 0;
}

void pulseConsumerControl(uint16_t usage) {
  releaseConsumerControl();
  if (usage == 0) {
    return;
  }

  ConsumerControl.press(usage);
  lastConsumerUsage = usage;
  consumerPulseReleaseAtMs = millis() + kConsumerPulseMs;
}

void updateConsumerPulse() {
  if (consumerPulseReleaseAtMs == 0) {
    return;
  }

  if (static_cast<int32_t>(millis() - consumerPulseReleaseAtMs) >= 0) {
    releaseConsumerControl();
  }
}

void releaseAllOutputs() {
  releaseMouseButtons();
  releaseConsumerControl();
  Keyboard.releaseAll();
  lastButtonMask = 0;
}

void applyMouseProfile(uint8_t buttonMask) {
  const bool leftClick = (buttonMask & kButtonTouchpad) || (buttonMask & kButtonMenu) || touchDragActive;
  const bool rightClick = (buttonMask & kButtonTouchpad2) || (buttonMask & kButtonAirplay);

  if (leftClick) {
    Mouse.press(MOUSE_LEFT);
  } else {
    Mouse.release(MOUSE_LEFT);
  }

  if (rightClick) {
    Mouse.press(MOUSE_RIGHT);
  } else {
    Mouse.release(MOUSE_RIGHT);
  }

  uint16_t usage = 0;
  if (buttonMask & kButtonVolumeUp) {
    usage = CONSUMER_CONTROL_VOLUME_INCREMENT;
  } else if (buttonMask & kButtonVolumeDown) {
    usage = CONSUMER_CONTROL_VOLUME_DECREMENT;
  } else if (buttonMask & kButtonPlayPause) {
    usage = CONSUMER_CONTROL_PLAY_PAUSE;
  }

  if (usage != lastConsumerUsage) {
    releaseConsumerControl();
    if (usage != 0) {
      ConsumerControl.press(usage);
      lastConsumerUsage = usage;
    }
  }
}

void setKeyboardState(uint8_t key, bool pressed) {
  if (pressed) {
    Keyboard.press(key);
  } else {
    Keyboard.release(key);
  }
}

void triggerPlayAction() {
  if (!hidReady()) {
    return;
  }

  switch (currentProfile) {
    case InputProfile::Mouse:
    case InputProfile::Media:
      pulseConsumerControl(CONSUMER_CONTROL_PLAY_PAUSE);
      break;
    case InputProfile::PowerPoint:
      Keyboard.press(KEY_F5);
      delay(20);
      Keyboard.release(KEY_F5);
      break;
  }
}

void applyPowerPointProfile(uint8_t buttonMask) {
  releaseMouseButtons();
  releaseConsumerControl();

  setKeyboardState(KEY_LEFT_ARROW, (buttonMask & kButtonMenu) || (buttonMask & kButtonVolumeDown) ||
                                       (buttonMask & kButtonTouchpad2));
  setKeyboardState(KEY_RIGHT_ARROW, (buttonMask & kButtonAirplay) || (buttonMask & kButtonVolumeUp) ||
                                        (buttonMask & kButtonTouchpad));
  setKeyboardState(KEY_F5, buttonMask & kButtonPlayPause);
  setKeyboardState(KEY_ESC, buttonMask & kButtonSiri);
}

void applyMediaProfile(uint8_t buttonMask) {
  releaseMouseButtons();
  Keyboard.releaseAll();

  uint16_t usage = 0;
  bool pulseUsage = false;
  if (buttonMask & kButtonVolumeUp) {
    usage = CONSUMER_CONTROL_VOLUME_INCREMENT;
  } else if (buttonMask & kButtonVolumeDown) {
    usage = CONSUMER_CONTROL_VOLUME_DECREMENT;
  } else if (buttonMask & kButtonPlayPause) {
    usage = CONSUMER_CONTROL_PLAY_PAUSE;
    pulseUsage = true;
  } else if (buttonMask & kButtonAirplay) {
    usage = SIRI_SWAP_MEDIA_NEXT_PREV ? CONSUMER_CONTROL_SCAN_PREVIOUS : CONSUMER_CONTROL_SCAN_NEXT;
    pulseUsage = true;
  } else if (buttonMask & kButtonMenu) {
    usage = SIRI_SWAP_MEDIA_NEXT_PREV ? CONSUMER_CONTROL_SCAN_NEXT : CONSUMER_CONTROL_SCAN_PREVIOUS;
    pulseUsage = true;
  }

  if (pulseUsage) {
    pulseConsumerControl(usage);
    return;
  }

  if (usage != lastConsumerUsage) {
    releaseConsumerControl();
    if (usage != 0) {
      ConsumerControl.press(usage);
      lastConsumerUsage = usage;
    }
  }
}

void cycleProfile(int direction) {
  const int count = 3;
  int next = static_cast<int>(currentProfile) + direction;
  if (next < 0) {
    next += count;
  } else if (next >= count) {
    next -= count;
  }
  currentProfile = static_cast<InputProfile>(next);
  releaseAllOutputs();
  resetTouchTracking();
  debugLogf("Switched input profile to %s\n", profileName(currentProfile));
}

void applyButtonState(uint8_t buttonMask) {
  if (!hidReady()) {
    lastButtonMask = buttonMask;
    debugLogf("HID not ready yet, deferring button state 0x%02X\n", buttonMask);
    return;
  }

  switch (currentProfile) {
    case InputProfile::Mouse:
      Keyboard.releaseAll();
      applyMouseProfile(buttonMask);
      break;
    case InputProfile::PowerPoint:
      applyPowerPointProfile(buttonMask);
      break;
    case InputProfile::Media:
      applyMediaProfile(buttonMask);
      break;
  }

  lastButtonMask = buttonMask;
  setLedState(LedState::Activity, 120);
  debugLogf("Applied %s profile HID state for buttons 0x%02X\n", profileName(currentProfile), buttonMask);
}

void handleBatteryEvent(uint8_t percent) {
  debugLogf("Battery: %u%%\n", percent);
}

void handlePowerEvent(uint8_t rawState) {
  switch (rawState) {
    case kPowerCharging:
      debugLogln("Power: charging");
      break;
    case kPowerDischarging:
      debugLogln("Power: discharging");
      break;
    case kPowerPluggedIn:
      debugLogln("Power: plugged in");
      break;
    default:
      debugLogf("Power: unknown state 0x%02X\n", rawState);
      break;
  }
}

void resetTouchTracking() {
  previousTouchX = INT16_MIN;
  previousTouchY = INT16_MIN;
  previousTouchCount = 0;
  filteredPointerDx = 0;
  filteredPointerDy = 0;
  pointerRemainderX = 0;
  pointerRemainderY = 0;
  filteredScrollWheel = 0;
  filteredScrollPan = 0;
  scrollRemainderWheel = 0;
  scrollRemainderPan = 0;
  tapCandidateActive = false;
  tapCandidateMoved = false;
  touchDragActive = false;
  tapCandidateStartMs = 0;
  tapCandidateStartX = INT16_MIN;
  tapCandidateStartY = INT16_MIN;
}

struct DecodedFinger {
  int16_t x;
  int16_t y;
  uint8_t pressure;
};

DecodedFinger decodeFinger(const uint8_t *fingerData) {
  return {
      static_cast<int16_t>((fingerData[0] + 255 * (fingerData[1] & 0x07) - 230) / 15),
      static_cast<int16_t>(((fingerData[2] & 0x80) ? fingerData[2] : fingerData[2] + 255) - 188),
      fingerData[5],
  };
}

bool isFingerActive(const DecodedFinger &finger) {
  return finger.pressure >= kMinTouchPressure;
}

bool isLikelyEdgeTouch(const DecodedFinger &finger) {
  if (!SIRI_EDGE_FILTER) {
    return false;
  }
  return finger.x <= kTouchEdgeMarginX || finger.x >= (kTouchMaxX - kTouchEdgeMarginX) ||
         finger.y <= kTouchEdgeMarginY || finger.y >= (kTouchMaxY - kTouchEdgeMarginY);
}

int applyPointerResponseFixed(int delta) {
  const int magnitude = abs(delta);
  int scaled = delta * kPointerBaseGain;

  if (magnitude > kPointerAccelerationThreshold) {
    const int extra = (magnitude - kPointerAccelerationThreshold) / kPointerAccelerationDivisor;
    scaled += delta > 0 ? extra : -extra;
  }

  return scaled;
}

int applyScrollResponseFixed(int delta) {
  const int magnitude = abs(delta);
  int scaled = delta * kScrollBaseGain;

  if (magnitude > kScrollAccelerationThreshold) {
    const int extra = (magnitude - kScrollAccelerationThreshold) * kScrollAccelerationBoost;
    scaled += delta > 0 ? extra : -extra;
  }

  return scaled;
}

int consumeScaledDelta(int scaledDelta, int &remainder, int scale) {
  remainder += scaledDelta;
  const int wholeDelta = remainder / scale;
  remainder -= wholeDelta * scale;
  return wholeDelta;
}

bool mouseAllowedForProfile() {
  return currentProfile == InputProfile::Mouse || currentProfile == InputProfile::PowerPoint;
}

void beginTapCandidate(const DecodedFinger &finger) {
  tapCandidateActive = true;
  tapCandidateMoved = false;
  tapCandidateStartMs = millis();
  tapCandidateStartX = finger.x;
  tapCandidateStartY = finger.y;

  if (lastTapReleaseMs != 0 &&
      tapCandidateStartMs - lastTapReleaseMs <= kDoubleTapWindowMs &&
      abs(finger.x - lastTapReleaseX) <= 6 &&
      abs(finger.y - lastTapReleaseY) <= 6) {
    touchDragActive = true;
    if (hidReady() && mouseAllowedForProfile()) {
      Mouse.press(MOUSE_LEFT);
    }
    debugLogln("Double-tap drag armed");
  }
}

void updateTapCandidate(const DecodedFinger &finger) {
  if (!tapCandidateActive) {
    return;
  }
  if (abs(finger.x - tapCandidateStartX) > 4 || abs(finger.y - tapCandidateStartY) > 4) {
    tapCandidateMoved = true;
  }
}

void maybeEmitTapClick() {
  if (touchDragActive) {
    if (hidReady()) {
      Mouse.release(MOUSE_LEFT);
    }
    touchDragActive = false;
    tapCandidateActive = false;
    tapCandidateMoved = false;
    debugLogln("Double-tap drag released");
    return;
  }

  if (!tapCandidateActive || tapCandidateMoved || !mouseAllowedForProfile() || !hidReady()) {
    tapCandidateActive = false;
    return;
  }

  if (millis() - tapCandidateStartMs <= kTapMaxDurationMs) {
    Mouse.click(MOUSE_LEFT);
    lastTapReleaseMs = millis();
    lastTapReleaseX = tapCandidateStartX;
    lastTapReleaseY = tapCandidateStartY;
    debugLogln("Tap click emitted");
  }
  tapCandidateActive = false;
}

void handleSingleFingerGesture(const DecodedFinger &finger) {
  if (!mouseAllowedForProfile()) {
    return;
  }

  if (isLikelyEdgeTouch(finger)) {
    if (shouldLogTouchNow()) {
      debugLogf("Ignoring edge touch x=%d y=%d pressure=%u\n", finger.x, finger.y, finger.pressure);
    }
    return;
  }

  if (millis() < scrollReleaseGateUntilMs) {
    previousTouchX = finger.x;
    previousTouchY = finger.y;
    previousTouchCount = 1;
    return;
  }

  if (previousTouchCount != 1) {
    beginTapCandidate(finger);
  } else {
    updateTapCandidate(finger);
  }

  if (previousTouchX != INT16_MIN && previousTouchY != INT16_MIN && hidReady()) {
    const int rawDx = (finger.x - previousTouchX) * SIRI_TOUCH_SENSITIVITY;
    const int rawDy = (previousTouchY - finger.y) * SIRI_TOUCH_SENSITIVITY;

    filteredPointerDx =
        (filteredPointerDx * kPointerSmoothingNumerator + rawDx * (kPointerSmoothingDenominator - kPointerSmoothingNumerator)) /
        kPointerSmoothingDenominator;
    filteredPointerDy =
        (filteredPointerDy * kPointerSmoothingNumerator + rawDy * (kPointerSmoothingDenominator - kPointerSmoothingNumerator)) /
        kPointerSmoothingDenominator;

    const int responseDx = applyPointerResponseFixed(filteredPointerDx);
    const int responseDy = applyPointerResponseFixed(filteredPointerDy);
    const int outputDx = consumeScaledDelta(responseDx, pointerRemainderX, kPointerResponseScale);
    const int outputDy = consumeScaledDelta(responseDy, pointerRemainderY, kPointerResponseScale);
    const int8_t clampedDx = clampMouseDelta(outputDx);
    const int8_t clampedDy = clampMouseDelta(outputDy);

    Mouse.move(clampedDx, clampedDy);
    setLedState(LedState::Activity, 80);
    if ((clampedDx != 0 || clampedDy != 0) && shouldLogTouchNow()) {
      debugLogf("Touch move x=%d y=%d pressure=%u raw=(%d,%d) smooth=(%d,%d) response=(%d,%d) out=(%d,%d)\n",
                finger.x, finger.y, finger.pressure,
                rawDx, rawDy,
                filteredPointerDx, filteredPointerDy,
                responseDx, responseDy,
                clampedDx, clampedDy);
    }
  }

  previousTouchX = finger.x;
  previousTouchY = finger.y;
  previousTouchCount = 1;
}

void handleTwoFingerGesture(const DecodedFinger &fingerA, const DecodedFinger &fingerB) {
  if (!mouseAllowedForProfile()) {
    return;
  }

  tapCandidateActive = false;

  if (isLikelyEdgeTouch(fingerA) || isLikelyEdgeTouch(fingerB)) {
    return;
  }

  const int16_t centerX = static_cast<int16_t>((fingerA.x + fingerB.x) / 2);
  const int16_t centerY = static_cast<int16_t>((fingerA.y + fingerB.y) / 2);

  if (previousTouchCount == 2 && previousTouchX != INT16_MIN && previousTouchY != INT16_MIN && hidReady()) {
    const int rawPan = (centerX - previousTouchX) * kScrollSensitivity;
    const int rawWheel = (previousTouchY - centerY) * kScrollSensitivity;
    filteredScrollPan =
        (filteredScrollPan * kScrollSmoothingNumerator + rawPan * (kScrollSmoothingDenominator - kScrollSmoothingNumerator)) /
        kScrollSmoothingDenominator;
    filteredScrollWheel =
        (filteredScrollWheel * kScrollSmoothingNumerator + rawWheel * (kScrollSmoothingDenominator - kScrollSmoothingNumerator)) /
        kScrollSmoothingDenominator;
    const int responseWheel = applyScrollResponseFixed(filteredScrollWheel);
    const int responsePan = applyScrollResponseFixed(filteredScrollPan);
    const int outWheel = consumeScaledDelta(responseWheel, scrollRemainderWheel, kScrollResponseScale);
    const int outPan = consumeScaledDelta(responsePan, scrollRemainderPan, kScrollResponseScale);
    Mouse.move(0, 0, clampMouseDelta(outWheel), clampMouseDelta(outPan));
    setLedState(LedState::Activity, 80);
    if ((outWheel != 0 || outPan != 0) && shouldLogTouchNow()) {
      debugLogf("Multi-touch center=(%d,%d) a=(%d,%d,%u) b=(%d,%d,%u) raw=(%d,%d) smooth=(%d,%d) response=(%d,%d) out=(%d,%d)\n",
                centerX, centerY,
                fingerA.x, fingerA.y, fingerA.pressure,
                fingerB.x, fingerB.y, fingerB.pressure,
                rawWheel, rawPan,
                filteredScrollWheel, filteredScrollPan,
                responseWheel, responsePan,
                clampMouseDelta(outWheel), clampMouseDelta(outPan));
    }
  }

  previousTouchX = centerX;
  previousTouchY = centerY;
  previousTouchCount = 2;
}

void handleTouchpadEvent(const uint8_t *data, size_t length) {
  const DecodedFinger fingerA = decodeFinger(data + 6);
  const bool fingerAActive = isFingerActive(fingerA);
  const bool fingerAUsable = fingerAActive && !isLikelyEdgeTouch(fingerA);

  if (length == 13) {
    if (!fingerAActive) {
      if (shouldLogTouchNow()) {
        debugLogf("Touch release at raw x=%u zone=%u y=%u pressure=%u\n", data[6], data[7], data[8], fingerA.pressure);
      }
      maybeEmitTapClick();
      resetTouchTracking();
      return;
    }

    if (fingerAUsable) {
      handleSingleFingerGesture(fingerA);
    }
    return;
  }

  const DecodedFinger fingerB = decodeFinger(data + 13);
  const bool fingerBActive = isFingerActive(fingerB);
  const bool fingerBUsable = fingerBActive && !isLikelyEdgeTouch(fingerB);

  if (!fingerAActive && !fingerBActive) {
    if (shouldLogTouchNow()) {
      debugLogf("Multi-touch release a=(%d,%d,%u) b=(%d,%d,%u)\n",
                fingerA.x, fingerA.y, fingerA.pressure,
                fingerB.x, fingerB.y, fingerB.pressure);
    }
    scrollReleaseGateUntilMs = millis() + kScrollReleaseGateMs;
    maybeEmitTapClick();
    resetTouchTracking();
    return;
  }

  if (fingerAUsable && fingerBUsable) {
    handleTwoFingerGesture(fingerA, fingerB);
    return;
  }

  const DecodedFinger *activeFinger = nullptr;
  if (fingerAUsable) {
    activeFinger = &fingerA;
  } else if (fingerBUsable) {
    activeFinger = &fingerB;
  }

  if (activeFinger == nullptr) {
    return;
  }

  if (previousTouchCount == 2) {
    if (shouldLogTouchNow()) {
      debugLogf("Ending multi-touch without single-finger handoff, active=(%d,%d,%u)\n",
                activeFinger->x, activeFinger->y, activeFinger->pressure);
    }
    scrollReleaseGateUntilMs = millis() + kScrollReleaseGateMs;
    resetTouchTracking();
    return;
  }

  if (shouldLogTouchNow()) {
    debugLogf("Ignoring ghost finger, active=(%d,%d,%u)\n",
              activeFinger->x, activeFinger->y, activeFinger->pressure);
  }
  handleSingleFingerGesture(*activeFinger);
}

void handleInputEvent(const uint8_t *data, size_t length) {
  if (length < 2) {
    debugLogf("Ignoring short input packet len=%u\n", static_cast<unsigned>(length));
    return;
  }

  uint8_t buttonMask = data[1];
  const uint8_t rawButtonMask = buttonMask;
  const bool playHeld = buttonMask & kButtonPlayPause;
  const bool volumeUpHeld = buttonMask & kButtonVolumeUp;
  const bool volumeDownHeld = buttonMask & kButtonVolumeDown;
  const bool playReleased = (lastRawButtonMask & kButtonPlayPause) && !playHeld;
  const bool volumeUpPressed = !(lastRawButtonMask & kButtonVolumeUp) && volumeUpHeld;
  const bool volumeDownPressed = !(lastRawButtonMask & kButtonVolumeDown) && volumeDownHeld;

  if (profileSwitchSuppressUntilRelease) {
    releaseAllOutputs();
    if (buttonMask == kButtonReleased) {
      profileSwitchSuppressUntilRelease = false;
      debugLogln("Profile switch combo released");
    }
    lastRawButtonMask = rawButtonMask;
    return;
  }

  if (playHeld) {
    if (!profileSelectionActive) {
      profileSelectionActive = true;
      profileSelectionModifierUsed = false;
      releaseAllOutputs();
      debugLogln("Entered profile selection mode");
    }

    if (volumeUpPressed) {
      profileSelectionModifierUsed = true;
      cycleProfile(1);
    } else if (volumeDownPressed) {
      profileSelectionModifierUsed = true;
      cycleProfile(-1);
    }

    lastRawButtonMask = rawButtonMask;
    return;
  }

  if (playReleased && profileSelectionActive) {
    profileSelectionActive = false;
    profileSwitchSuppressUntilRelease = true;
    debugLogln("Exited profile selection mode");
    if (!profileSelectionModifierUsed) {
      triggerPlayAction();
      debugLogln("Play action emitted after profile-selection release");
    }
    profileSelectionModifierUsed = false;
    lastRawButtonMask = rawButtonMask;
    return;
  }

  buttonMask = static_cast<uint8_t>(buttonMask & ~kButtonPlayPause);

  if (data[0] == 2 && (buttonMask & kButtonTouchpad)) {
    buttonMask = static_cast<uint8_t>(buttonMask + (kButtonTouchpad2 - kButtonTouchpad));
    debugLogf("Detected 2-finger touch click, remapped buttons to 0x%02X\n", buttonMask);
  }

  if (buttonMask != lastButtonMask) {
    if (buttonMask == kButtonReleased) {
      releaseAllOutputs();
      debugLogln("Released all HID outputs");
    } else {
      applyButtonState(buttonMask);
    }
    debugLogf("Buttons changed: 0x%02X\n", buttonMask);
  }

  if ((length == 13 || length == 20) && data[2] == kTouchEventMarker) {
    handleTouchpadEvent(data, length);
  }

  lastRawButtonMask = rawButtonMask;
}

void notificationCallback(BLERemoteCharacteristic *characteristic, uint8_t *data, size_t length, bool) {
  const uint16_t handle = characteristic->getHandle();
  if (handle == kInputReportHandle) {
    handleInputEvent(data, length);
    return;
  }

  if (batteryCharacteristic != nullptr && handle == batteryCharacteristic->getHandle() && length >= 1) {
    handleBatteryEvent(data[0]);
    return;
  }

  if (powerCharacteristic != nullptr && handle == powerCharacteristic->getHandle() && length >= 1) {
    handlePowerEvent(data[0]);
  }
}

BLERemoteCharacteristic *getCharacteristicByHandle(BLERemoteService *service, uint16_t handle) {
  if (service == nullptr) {
    return nullptr;
  }

  auto *characteristics = service->getCharacteristicsByHandle();
  auto it = characteristics->find(handle);
  if (it == characteristics->end()) {
    return nullptr;
  }
  return it->second;
}

class RemoteSecurityCallbacks : public BLESecurityCallbacks {
 public:
  uint32_t onPassKeyRequest() override {
    debugLogln("BLE security: passkey requested, returning 000000");
    return 0;
  }

  void onPassKeyNotify(uint32_t passKey) override {
    debugLogf("BLE security: passkey %06lu\n", static_cast<unsigned long>(passKey));
  }

  bool onSecurityRequest() override {
    debugLogln("BLE security: accepting security request");
    return true;
  }

  void onAuthenticationComplete(esp_ble_auth_cmpl_t authCmpl) override {
    debugLogf("BLE security: auth %s\n", authCmpl.success ? "ok" : "failed");
  }

  bool onConfirmPIN(uint32_t pin) override {
    debugLogf("BLE security: confirming PIN %06lu\n", static_cast<unsigned long>(pin));
    return true;
  }
};

RemoteSecurityCallbacks securityCallbacks;

class RemoteClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient *) override {
    debugLogln("BLE connected");
  }

  void onDisconnect(BLEClient *) override {
    debugLogln("BLE disconnected");
    bleConnected = false;
    scanRunning = false;
    shouldConnect = false;
    rescanRequested = true;
    reconnectBlockedUntilMs = 0;
    lastDisconnectMs = millis();
    logReconnectPhase("disconnect", lastDisconnectMs);
    if (!haveRemoteCharacteristicCache()) {
      markBleSessionDirty("disconnect without a reusable GATT cache");
    }
  }
};

RemoteClientCallbacks clientCallbacks;

class RemoteScannerCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    if (bleConnected || shouldConnect) {
      return;
    }

    const std::string address = advertisedDevice.getAddress().toString();
    debugLogf("Scan result addr=%s name=%s rssi=%d\n",
              address.c_str(),
              advertisedDevice.haveName() ? advertisedDevice.getName().c_str() : "<none>",
              advertisedDevice.getRSSI());
    if (strcasecmp(address.c_str(), kRemoteMac) != 0) {
      return;
    }

    debugLogf("Found Siri Remote advertisement from %s\n", address.c_str());
    BLEDevice::getScan()->stop();
    scanRunning = false;
    lastRemoteAdvertisementMs = millis();
    logReconnectPhase("advertisement", lastRemoteAdvertisementMs);

    if (remoteDevice != nullptr) {
      delete remoteDevice;
    }
    remoteDevice = new BLEAdvertisedDevice(advertisedDevice);
    shouldConnect = true;
  }
};

RemoteScannerCallbacks scannerCallbacks;

void beginBleSecurity() {
  BLESecurity security;
  security.setAuthenticationMode(ESP_LE_AUTH_REQ_SC_BOND);
  security.setCapability(ESP_IO_CAP_NONE);
  security.setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  security.setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  security.setKeySize(16);

  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
  BLEDevice::setSecurityCallbacks(&securityCallbacks);
}

void resetBleSession() {
  BLEScan *scan = BLEDevice::getScan();
  if (scanRunning) {
    scan->stop();
    scanRunning = false;
  }
  scan->clearResults();

  if (remoteDevice != nullptr) {
    delete remoteDevice;
    remoteDevice = nullptr;
  }

  if (bleClient != nullptr) {
    delete bleClient;
    bleClient = nullptr;
  }

  invalidateRemoteCharacteristics();
  bleConnected = false;
  shouldConnect = false;
  bleSessionDirty = false;
}

void startScan() {
  if (scanRunning || hasRemoteMac() == false) {
    debugLogf("Skipping scan start, scanRunning=%u hasRemoteMac=%u\n", scanRunning, hasRemoteMac());
    return;
  }

  BLEScan *scan = BLEDevice::getScan();
  if (bleSessionDirty) {
    debugLogln("Resetting BLE session before scan");
    resetBleSession();
  } else {
    scan->clearResults();
  }
  scan->setAdvertisedDeviceCallbacks(&scannerCallbacks, false, true);
  scan->setActiveScan(true);
  // Arduino BLEScan expects milliseconds here, so keep the reconnect scanner
  // in a tight continuous window to minimize wake-to-connect latency.
  scan->setInterval(kScanIntervalMs);
  scan->setWindow(kScanWindowMs);
  lastRemoteAdvertisementMs = 0;
  lastScanStartedMs = millis();
  logReconnectPhase("scan-start", lastScanStartedMs);

  debugLogf("Scanning for Siri Remote %s\n", kRemoteMac);
  scanRunning = scan->start(0, nullptr, false);
  debugLogf("Scan start result: %u (reconnect blocked until %lu ms)\n",
            scanRunning,
            static_cast<unsigned long>(reconnectBlockedUntilMs));
  setLedState(LedState::Scanning);
}

bool resolveRemoteCharacteristics(bool &usedCache) {
  usedCache = haveRemoteCharacteristicCache();
  if (usedCache) {
    debugLogln("Reusing cached Siri Remote GATT characteristics");
    return true;
  }

  BLERemoteService *batteryService = bleClient->getService(BLEUUID(kBatteryServiceUuid));
  BLERemoteService *hidService = bleClient->getService(BLEUUID(kHidServiceUuid));
  if (batteryService == nullptr || hidService == nullptr) {
    debugLogln("Required GATT services not found");
    return false;
  }

  debugLogf("Found services: battery handle=0x%04X hid handle=0x%04X\n",
            batteryService->getHandle(), hidService->getHandle());

  batteryCharacteristic = batteryService->getCharacteristic(BLEUUID(kBatteryLevelUuid));
  powerCharacteristic = batteryService->getCharacteristic(BLEUUID(kPowerStateUuid));
  inputCharacteristic = getCharacteristicByHandle(hidService, kInputReportHandle);
  enableInputCharacteristic = getCharacteristicByHandle(hidService, kEnableInputHandle);

  if (!haveRemoteCharacteristicCache()) {
    invalidateRemoteCharacteristics();
    return false;
  }

  debugLogf("Characteristic handles: battery=0x%04X power=0x%04X input=0x%04X enable=0x%04X\n",
            batteryCharacteristic->getHandle(),
            powerCharacteristic->getHandle(),
            inputCharacteristic->getHandle(),
            enableInputCharacteristic->getHandle());
  return true;
}

bool connectToRemote() {
  if (remoteDevice == nullptr) {
    return false;
  }

  setLedState(LedState::Connecting);
  lastConnectStartedMs = millis();
  logReconnectPhase("connect-start", lastConnectStartedMs);
  debugLogf("Connecting to %s\n", remoteDevice->getAddress().toString().c_str());

  if (bleClient == nullptr) {
    bleClient = BLEDevice::createClient();
    bleClient->setClientCallbacks(&clientCallbacks);
    debugLogln("Created BLE client");
  }

  if (!bleClient->connect(remoteDevice)) {
    debugLogln("BLE connect failed");
    markBleSessionDirty("connect failed");
    setLedState(LedState::Error, 700);
    return false;
  }

  logReconnectPhase("connected", millis());
  bleClient->setMTU(104);
  debugLogln("Requested MTU 104");
  bool usedCachedCharacteristics = false;
  if (!resolveRemoteCharacteristics(usedCachedCharacteristics)) {
    debugLogln("Required GATT characteristics not found");
    markBleSessionDirty("characteristic resolution failed");
    bleClient->disconnect();
    setLedState(LedState::Error, 700);
    return false;
  }

  debugLogf("Using %s GATT path for reconnect\n", usedCachedCharacteristics ? "cached" : "discovered");
  inputCharacteristic->registerForNotify(notificationCallback);
  logReconnectPhase("input-notify-registered", millis());
  enableInputCharacteristic->writeValue(kMagicEnableInput, true);
  logReconnectPhase("input-enabled", millis());
  debugLogf("Enabled input path with magic byte 0x%02X\n", kMagicEnableInput);

  bleConnected = true;
  resetTouchTracking();
  releaseAllOutputs();
  setLedState(LedState::Connected);
  const uint32_t readyMs = millis();
  debugLogf("Siri Remote input path ready at %lu ms\n", static_cast<unsigned long>(readyMs));
  if (lastDisconnectMs != 0 && lastScanStartedMs >= lastDisconnectMs &&
      lastRemoteAdvertisementMs >= lastScanStartedMs && lastConnectStartedMs >= lastRemoteAdvertisementMs) {
    debugLogf("Reconnect timing summary: disconnect->scan=%lu ms scan->advertisement=%lu ms advertisement->connect=%lu ms connect->ready=%lu ms total=%lu ms\n",
              static_cast<unsigned long>(lastScanStartedMs - lastDisconnectMs),
              static_cast<unsigned long>(lastRemoteAdvertisementMs - lastScanStartedMs),
              static_cast<unsigned long>(lastConnectStartedMs - lastRemoteAdvertisementMs),
              static_cast<unsigned long>(readyMs - lastConnectStartedMs),
              static_cast<unsigned long>(readyMs - lastDisconnectMs));
  }

  batteryCharacteristic->registerForNotify(notificationCallback);
  powerCharacteristic->registerForNotify(notificationCallback);
  debugLogln("Siri Remote bridge ready");
  return true;
}

void initUsbHid() {
  HID.begin();
  Mouse.begin();
  ConsumerControl.begin();
  Keyboard.begin();
  const bool usbStarted = USB.begin();
  debugLogf("USB device stack start: %s\n", usbStarted ? "ok" : "failed");
  debugLogln("USB HID interfaces initialized");
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(50);

  strip.begin();
  strip.setBrightness(kBrightness);
  setLedState(LedState::Idle);

  debugLogln("");
  debugLogln("ESP32 Siri Remote bridge booting");
  debugLogf("Configured MAC: %s\n", kRemoteMac);
  debugLogf("Touch sensitivity: %d\n", SIRI_TOUCH_SENSITIVITY);
  debugLogf("Edge filter: %s\n", SIRI_EDGE_FILTER ? "on" : "off");
  debugLogf("Default input profile: %s\n", profileName(currentProfile));
  debugLogln("Serial logging active at 115200 baud");

  if (!hasRemoteMac()) {
    debugLogln("Set SIRI_REMOTE_MAC in platformio.ini before flashing.");
    setLedState(LedState::Error);
  }

  initUsbHid();

  BLEDevice::init("");
  debugLogln("BLE stack initialized");
  beginBleSecurity();
  debugLogln("BLE security configured");

  startScan();
}

void loop() {
  updateLedState();
  updateConsumerPulse();
  const uint32_t now = millis();

  if (hidReady() && !usbReadyLogged) {
    usbReadyLogged = true;
    debugLogln("USB HID ready");
  }

  if (rescanRequested && !bleConnected) {
    rescanRequested = false;
    reconnectBlockedUntilMs = 0;
    debugLogf("Processing disconnect recovery at %lu ms\n", static_cast<unsigned long>(now));
    resetTouchTracking();
    releaseAllOutputs();
    setLedState(LedState::Scanning);
    startScan();
  }

  if (!bleConnected && shouldConnect && static_cast<int32_t>(now - reconnectBlockedUntilMs) >= 0) {
    shouldConnect = false;
    debugLogf("Connect gate opened at %lu ms\n", static_cast<unsigned long>(now));
    if (!connectToRemote()) {
      reconnectBlockedUntilMs = now + kReconnectDelayMs;
      debugLogln("Connect attempt failed, returning to scan");
      startScan();
    }
  }

  if (!bleConnected && !shouldConnect && !scanRunning && static_cast<int32_t>(now - reconnectBlockedUntilMs) >= 0) {
    debugLogf("Reconnect gate open, starting scan at %lu ms\n", static_cast<unsigned long>(now));
    startScan();
  }

  if (!bleConnected && now - lastStatusPrintMs >= kStatusPrintIntervalMs) {
    lastStatusPrintMs = now;
    debugLogln("Waiting for Siri Remote advertisement. Press a remote button to wake it.");
  }

  delay(1);
}
