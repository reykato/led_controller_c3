#include <Arduino.h>
#include "config.h"  // Using configuration constants defined in config.h
#include "ESPNowClient.h"
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Adafruit_NeoPixel.h>

// Global LED state and brightness (initial values)
int brightness = 0;
int rgbBrightness = 0;

// Temperature is the proportion of warm white to cool white
// 0 is all cool white, 255 is all warm white
int temperature = 127;
uint8_t warmDuty = 0;
uint8_t coolDuty = 0;

bool ledOn = true;

ESPNowClient espNow;

IPAddress local_IP(192, 168, 68, 46);
IPAddress gateway(192, 168, 68, 1);
IPAddress subnet(255, 255, 255, 0);

#undef NUM_LEDS
#define NUM_LEDS 47
#define STAGGER_OFFSET 2
Adafruit_NeoPixel strip_r1(NUM_LEDS, PIN_DATA1, NEO_RGB + NEO_KHZ800);
Adafruit_NeoPixel strip_r2(NUM_LEDS, PIN_DATA2, NEO_RGB + NEO_KHZ800);
Adafruit_NeoPixel strip_l1(NUM_LEDS, PIN_DATA3, NEO_RGB + NEO_KHZ800);
Adafruit_NeoPixel strip_l2(NUM_LEDS, PIN_DATA4, NEO_RGB + NEO_KHZ800);
int cylonPos = 0;
int cylonDir = 1;

// Updates both LED channels based on ledOn and brightness
void updateWhiteLEDs(uint8_t warmDuty, uint8_t coolDuty) {
  uint8_t warmDutyCalcd = ledOn ? warmDuty : 0;
  uint8_t coolDutyCalcd = ledOn ? coolDuty : 0;
  ledcWrite(PWM_CHANNEL_0, coolDutyCalcd);
  ledcWrite(PWM_CHANNEL_1, warmDutyCalcd);
}

void onReceive(const uint8_t *mac, const uint8_t *data, size_t len) {
  if (len < 2) return;
  uint8_t command = data[0];
  uint8_t state = data[1];
  switch(command) {
    case COMMAND_BRIGHTNESS:
      brightness = state;
      break;
    case COMMAND_TOGGLE:
      ledOn = (state == 1);
      break;
    default:
      Serial.print("Unknown command received: 0x");
      Serial.println(command, HEX);
      break;
  }
  updateWhiteLEDs(warmDuty, coolDuty);
}

void initOTA() {
  ArduinoOTA.setHostname("ESP32-LED-Controller");
  ArduinoOTA.setPassword("pwd611493");
  ArduinoOTA.begin();
  Serial.println("OTA Initialized");
}

void initWiFi(const char* ssid, const char* password) {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(ssid, password);

  int timeout = 20;  // Timeout in seconds
  while (WiFi.status() != WL_CONNECTED && timeout > 0) {
      delay(1000);
      Serial.print(".");
      timeout--;
  }

  if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi connected!");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      Serial.println("Configuring OTA...");
      initOTA();
  } else {
      Serial.println("\nFailed to connect to WiFi.");
      ESP.restart();
  }
}

void setup() {
  Serial.begin(115200);

  initWiFi(WIFI_SSID, WIFI_PASSWORD);
  
  ledcSetup(PWM_CHANNEL_0, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_1, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(PIN_PWM1, PWM_CHANNEL_0);
  ledcAttachPin(PIN_PWM2, PWM_CHANNEL_1);

  // Calculate duty cycles based on temperature and brightness
  // temperature: 0 (cool) to 255 (warm)
  // For warm white (PIN_PWM1)
  warmDuty = map(temperature, 0, 255, 0, brightness);
  // For cool white (PIN_PWM2)
  coolDuty = map(temperature, 0, 255, brightness, 0);

  updateWhiteLEDs(warmDuty, coolDuty);

  // Debug output
  Serial.println("Temperature: " + String(temperature));
  Serial.println("Warm duty: " + String(warmDuty));
  Serial.println("Cool duty: " + String(coolDuty));


  if (!espNow.begin()) {
    Serial.println("Failed to initialize ESP-NOW client");
  }

  espNow.setReceiveCallback(onReceive);

  strip_r1.begin();
  strip_r1.fill(strip_r1.Color(255, 180, 170), 0, NUM_LEDS);
  strip_r1.setBrightness(rgbBrightness);
  strip_r1.show();
  
  strip_r2.begin();
  strip_r2.fill(strip_r2.Color(255, 180, 170), 0, NUM_LEDS);
  strip_r2.setBrightness(rgbBrightness);
  strip_r2.show();

  strip_l1.begin();
  strip_l1.fill(strip_l1.Color(255, 180, 170), 0, NUM_LEDS);
  strip_l1.setBrightness(rgbBrightness);
  strip_l1.show();

  strip_l2.begin();
  strip_l2.fill(strip_l2.Color(255, 180, 170), 0, NUM_LEDS);
  strip_l2.setBrightness(rgbBrightness);
  strip_l2.show();

}

// New function: updateRainbowEffect() performs the rainbow animation using variable intervals
void updateRainbowEffect() {
  static unsigned long lastRainbowUpdate = 0;
  static uint8_t hue = 0;
  static unsigned long rainbowInterval = RAINBOW_INTERVAL_HIGH;
  static bool droppingInterval = false;
  static unsigned long waitStart = 0;
  static unsigned long waitDuration = 0;
  static bool isWaiting = false;
  static float transitionProgress = 0.0;
  static const float transitionSpeed = 0.005;  // Controls transition speed
  
  unsigned long now = millis();
  if (now - lastRainbowUpdate >= rainbowInterval) {
    lastRainbowUpdate = now;
    
    // Clear both strips
    strip_r1.clear();
    strip_r2.clear();
    strip_l1.clear();
    strip_l2.clear();
    
    // Fill first strip with a rainbow that shifts
    for (int i = 0; i < NUM_LEDS; i++) {
      uint8_t ledHue = hue + (i * 256 / NUM_LEDS);
      uint32_t color = strip_r1.ColorHSV(((uint16_t)ledHue << 8), 255, 255);
      strip_r1.setPixelColor(i, color);
      strip_l1.setPixelColor(i, color);
    }
    
    // Fill second strip with a similarly shifted rainbow, staggered by STAGGER_OFFSET
    for (int i = 0; i < NUM_LEDS; i++) {
      uint8_t ledHue = hue + (((i + STAGGER_OFFSET) % NUM_LEDS) * 256 / NUM_LEDS);
      uint32_t color = strip_r2.ColorHSV(((uint16_t)ledHue << 8), 255, 255);
      strip_r2.setPixelColor(i, color);
      strip_l2.setPixelColor(i, color);
    }
    
    // Update both strips
    strip_r1.show();
    strip_r2.show();
    strip_l1.show();
    strip_l2.show();
    
    hue--;
    
    // Handle state transitions and interval updates
    if (isWaiting) {
      // In waiting phase
      if (now - waitStart >= waitDuration) {
        isWaiting = false;
        droppingInterval = true; // Start dropping again after wait
        transitionProgress = 0.0;
      }
    } else if (droppingInterval) {
      // Dropping phase - smoother easing curve
      transitionProgress += transitionSpeed;
      if (transitionProgress >= 1.0) {
        droppingInterval = false;
        transitionProgress = 0.0;
      } else {
        // Smooth ease-in-out curve for dropping (high to low)
        float factor = transitionProgress < 0.5 ? 
                      4 * transitionProgress * transitionProgress * transitionProgress :
                      1 - pow(-2 * transitionProgress + 2, 3) / 2;
        rainbowInterval = RAINBOW_INTERVAL_HIGH - 
                         (factor * (RAINBOW_INTERVAL_HIGH - RAINBOW_INTERVAL_LOW));
      }
    } else {
      // Raising phase - smoother easing curve 
      transitionProgress += transitionSpeed;
      if (transitionProgress >= 1.0) {
        // Start waiting phase after reaching high interval
        waitDuration = random(6000, 10000); // Wait 6-10 seconds
        waitStart = now;
        isWaiting = true;
        transitionProgress = 0.0;
      } else {
        // Smooth ease-in-out curve for raising (low to high)
        float factor = transitionProgress < 0.5 ? 
                      4 * transitionProgress * transitionProgress * transitionProgress :
                      1 - pow(-2 * transitionProgress + 2, 3) / 2;
        rainbowInterval = RAINBOW_INTERVAL_LOW + 
                         (factor * (RAINBOW_INTERVAL_HIGH - RAINBOW_INTERVAL_LOW));
      }
    }
  }
}

void loop() {
    ArduinoOTA.handle();
    
    // Use the new function to update the rainbow effect
    // updateRainbowEffect();
}

