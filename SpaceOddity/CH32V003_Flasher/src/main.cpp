#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ch32v003_swio.h>

// =============================================================================
// Hardware Pins (Space Oddity Carrier PCB + ESP32-C3)
// =============================================================================
constexpr uint8_t PIN_OLED_SDA        = 21;
constexpr uint8_t PIN_OLED_SCL        = 20;
constexpr uint8_t SCREEN_WIDTH        = 128;
constexpr uint8_t SCREEN_HEIGHT       = 64;
constexpr uint8_t SCREEN_I2C_ADDR     = 0x3C;

constexpr uint8_t PIN_JOYSTICK_BUTTON = 10;
constexpr uint8_t PIN_UART_RX         = 2;  // Connected to CH32V003 PD5 (TX)
constexpr uint8_t PIN_UART_TX         = 8;  // Connected to CH32V003 PD6 (RX)

// PowerLink UART protocol bytes
constexpr uint8_t MSG_INIT            = 0xAE;
constexpr uint8_t MSG_READY           = 0xAB;
constexpr uint8_t MSG_BTN_MASK        = 0xF0;
constexpr uint8_t MSG_BTN_PREFIX      = 0xB0;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

enum class AppState {
  READY_TO_FLASH,
  FLASHING,
  WAIT_POWER_BTN,
  UART_OK,
  FLASH_ERROR
};

static AppState g_state = AppState::READY_TO_FLASH;
static bool g_uartConnected = false;
static uint8_t g_buttonStates = 0;
static int g_flashedPin = 7;
static uint32_t g_lastInitSendMs = 0;
static uint32_t g_lastDisplayUpdateMs = 0;

// =============================================================================
// Display Helpers
// =============================================================================
static void drawProgressScreen(const char *stage, int done, int total) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println(F("FLASHING CH32V003..."));
  display.drawFastHLine(0, 10, SCREEN_WIDTH, SSD1306_WHITE);

  display.setCursor(0, 16);
  display.println(stage);

  int pct = (total > 0) ? (done * 100) / total : 0;
  if (pct > 100) pct = 100;

  // Progress bar outline & fill
  const int barX = 4, barY = 30, barW = 120, barH = 12;
  display.drawRect(barX, barY, barW, barH, SSD1306_WHITE);
  int fillW = ((barW - 4) * pct) / 100;
  if (fillW > 0) {
    display.fillRect(barX + 2, barY + 2, fillW, barH - 4, SSD1306_WHITE);
  }

  display.setCursor(0, 48);
  display.printf("%d / %d B (%d%%)", done, total, pct);
  display.display();
}

static void drawCurrentStateScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  switch (g_state) {
    case AppState::READY_TO_FLASH:
      display.setCursor(0, 0);
      display.println(F("CH32V003 SWIO FLASHER"));
      display.drawFastHLine(0, 9, SCREEN_WIDTH, SSD1306_WHITE);
      display.setCursor(0, 13);
      display.println(F("UART: No response"));
      display.setCursor(0, 25);
      display.println(F("1.Jumper IO7 -> SWDIO"));
      display.println(F("  (or IO9 -> SWDIO)"));
      display.setCursor(0, 45);
      display.println(F("2.Click Joystick btn"));
      display.println(F("  to Flash CH32V003"));
      break;

    case AppState::WAIT_POWER_BTN:
      display.setCursor(0, 0);
      display.printf("FLASH SUCCESS! (IO%d)", g_flashedPin);
      display.drawFastHLine(0, 9, SCREEN_WIDTH, SSD1306_WHITE);
      display.setCursor(0, 13);
      display.println(F("7612 bytes verified!"));
      display.setCursor(0, 25);
      display.println(F("1.Remove SWDIO wire"));
      display.println(F("2.Press Carrier PCB"));
      display.println(F("  POWER button now"));
      display.println(F("  to verify UART..."));
      break;

    case AppState::UART_OK: {
      display.setCursor(0, 0);
      display.println(F("CH32V003 VERIFIED OK!"));
      display.drawFastHLine(0, 9, SCREEN_WIDTH, SSD1306_WHITE);
      display.setCursor(0, 13);
      display.println(F("UART Handshake: OK"));
      display.setCursor(0, 24);
      display.println(F("Carrier PCB Buttons:"));

      const char *labels[4] = {"PWR", "B0", "B1", "B2"};
      for (int i = 0; i < 4; i++) {
        int bx = i * 32;
        int by = 35;
        bool pressed = (g_buttonStates >> i) & 0x01;
        if (pressed) {
          display.fillRect(bx, by, 28, 13, SSD1306_WHITE);
          display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        } else {
          display.drawRect(bx, by, 28, 13, SSD1306_WHITE);
          display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        }
        int textX = bx + (i == 0 ? 5 : 8);
        display.setCursor(textX, by + 3);
        display.print(labels[i]);
      }
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 54);
      display.println(F("Ready for StockCode!"));
      break;
    }

    case AppState::FLASH_ERROR:
      display.setCursor(0, 0);
      display.println(F("SWIO CONNECT FAILED!"));
      display.drawFastHLine(0, 9, SCREEN_WIDTH, SSD1306_WHITE);
      display.setCursor(0, 13);
      display.println(F("Check jumper wire:"));
      display.println(F(" ESP32 IO7 or IO9"));
      display.println(F(" -> Carrier SWDIO"));
      display.setCursor(0, 41);
      display.println(F("Unplug USB+Batt 2s,"));
      display.println(F("click Joy to retry."));
      break;

    case AppState::FLASHING:
      break;
  }

  display.display();
}

// =============================================================================
// UART Polling (PowerLink protocol)
// =============================================================================
static void pollCarrierUart() {
  if (!g_uartConnected && (millis() - g_lastInitSendMs >= 300)) {
    g_lastInitSendMs = millis();
    Serial1.write(MSG_INIT);
  }

  while (Serial1.available()) {
    uint8_t b = (uint8_t)Serial1.read();
    if (b == MSG_READY) {
      if (!g_uartConnected) {
        g_uartConnected = true;
        Serial.println(F("[PowerLink] Handshake completed (MSG_READY 0xAB received)!"));
        g_state = AppState::UART_OK;
        drawCurrentStateScreen();
      }
    } else if ((b & MSG_BTN_MASK) == MSG_BTN_PREFIX) {
      g_uartConnected = true;
      g_buttonStates = b & 0x0F;
      Serial.printf("[PowerLink] Button state byte: 0x%02X (PWR=%d, B0=%d, B1=%d, B2=%d)\r\n",
                    b,
                    (g_buttonStates >> 0) & 1,
                    (g_buttonStates >> 1) & 1,
                    (g_buttonStates >> 2) & 1,
                    (g_buttonStates >> 3) & 1);
      if (g_state != AppState::FLASHING) {
        g_state = AppState::UART_OK;
        drawCurrentStateScreen();
      }
    }
  }
}

static void runFlashSequence(int forcedPin = -1) {
  g_state = AppState::FLASHING;
  drawProgressScreen("Connecting SWIO...", 0, (int)ch32v003_firmware_len);

  bool ok = false;
  if (forcedPin > 0) {
    g_swio_progress_cb = drawProgressScreen;
    ok = swio_flash_chip(forcedPin);
    g_swio_progress_cb = nullptr;
    if (ok) g_flashedPin = forcedPin;
  } else {
    ok = swio_flash_auto(drawProgressScreen, &g_flashedPin);
  }

  if (ok) {
    g_uartConnected = false;
    g_state = AppState::WAIT_POWER_BTN;
  } else {
    g_state = AppState::FLASH_ERROR;
  }
  drawCurrentStateScreen();
}

// =============================================================================
// Setup & Loop
// =============================================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_JOYSTICK_BUTTON, INPUT_PULLUP);

  Wire.setPins(PIN_OLED_SDA, PIN_OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_I2C_ADDR);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("CH32V003 SWIO FLASHER"));
  display.drawFastHLine(0, 9, SCREEN_WIDTH, SSD1306_WHITE);
  display.setCursor(0, 22);
  display.println(F("Checking CH32V003"));
  display.println(F("UART link..."));
  display.display();

  Serial1.begin(115200, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);

  uint32_t startMs = millis();
  while (millis() - startMs < 1500) {
    pollCarrierUart();
    if (g_uartConnected) break;
    delay(10);
  }

  Serial.println(F("\r\n====================================================="));
  Serial.println(F(" Space Oddity - CH32V003 Single-Wire (SWIO) Flasher"));
  Serial.println(F("====================================================="));
  Serial.println(F("Controls:"));
  Serial.println(F("  - Click the Joystick button on the robot to auto-flash (scans IO7 then IO9)"));
  Serial.println(F("  - Or send Serial commands: 'p'/'f' (IO7) or 'P'/'F' (IO9)"));

  drawCurrentStateScreen();
}

void loop() {
  pollCarrierUart();

  // Check Joystick button click (active low with simple debounce)
  static bool lastBtnHigh = true;
  static uint32_t lastBtnChangeMs = 0;
  bool btnHigh = (digitalRead(PIN_JOYSTICK_BUTTON) == HIGH);
  if (btnHigh != lastBtnHigh && (millis() - lastBtnChangeMs > 50)) {
    lastBtnChangeMs = millis();
    lastBtnHigh = btnHigh;
    if (!btnHigh) {
      Serial.println(F("[UI] Joystick button pressed -> starting SWIO flash..."));
      runFlashSequence(-1);
    }
  }

  // Also support Serial Monitor commands
  while (Serial.available()) {
    char cmd = (char)Serial.read();
    if (cmd == 'p') {
      swio_probe_chip(7);
    } else if (cmd == 'P') {
      swio_probe_chip(9);
    } else if (cmd == 'f') {
      runFlashSequence(7);
    } else if (cmd == 'F') {
      runFlashSequence(9);
    }
  }

  if (millis() - g_lastDisplayUpdateMs >= 250) {
    g_lastDisplayUpdateMs = millis();
    drawCurrentStateScreen();
  }
}

