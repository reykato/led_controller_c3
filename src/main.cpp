#include <Arduino.h>
#include "config.h"  // Using configuration constants defined in config.h
#include "ESPNowClient.h"
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Adafruit_NeoPixel.h>

ESPNowClient espNow;

IPAddress local_IP(192, 168, 68, 46);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 0, 0);


// Temperature is the proportion of warm white to cool white
// 0 is all cool white, 65535 is all warm white
uint16_t temperature = 32767;

int brightness = 255;
int rgbBrightness = 255;
uint16_t globalHue = 0;
bool ledOn = true;
float speedModifier = 3.0;
int ledMode = 0; // 0: Rainbow, 1: Hue, 2: White


// Forward declarations for all effect functions
void effectRainbow(void);
void effectHue(void);
void effectWhite(void);

// Define an array holding functions for each effect
void (*effects[])(void) = {
  &effectRainbow,
  &effectHue, // placeholder for effectHue - needs special handling due to parameters
  &effectWhite
};

Adafruit_NeoPixel strip_r1(NUM_LEDS, PIN_DATA1, NEO_RGB + NEO_KHZ800);
Adafruit_NeoPixel strip_r2(NUM_LEDS, PIN_DATA2, NEO_RGB + NEO_KHZ800);
Adafruit_NeoPixel strip_l1(NUM_LEDS, PIN_DATA3, NEO_RGB + NEO_KHZ800);
Adafruit_NeoPixel strip_l2(NUM_LEDS, PIN_DATA4, NEO_RGB + NEO_KHZ800);


// Updates both LED channels based on ledOn and brightness
void setWhiteLEDs(uint16_t warmDuty, uint16_t coolDuty) {
  uint16_t warmDutyCalcd = ledOn ? warmDuty : 0;
  uint16_t coolDutyCalcd = ledOn ? coolDuty : 0;
  ledcWrite(PWM_CHANNEL_0, coolDutyCalcd);
  ledcWrite(PWM_CHANNEL_1, warmDutyCalcd);
}

void onReceive(const uint8_t *mac, const uint8_t *data, size_t len) {
  if (len < 2) return; // Need at least command and 1 byte of state
  
  uint8_t command = data[0];
  uint16_t value;
  
  // Handle state value based on payload length
  if (len == 2) {
    // If only 1 byte of state, expand it to 16-bit range (0-255 to 0-65535)
    value = (uint16_t)data[1] << 8 | data[1];
  } else if (len >= 3) {
    // If 2 bytes of state, combine into 16-bit integer
    value = (uint16_t)data[1] << 8 | data[2];
  } else {
    return; // Invalid length
  }
  
  switch(command) {
    case COMMAND_BRIGHTNESS:
      brightness = value >> 8; // Scale back to 0-255 for brightness
      rgbBrightness = brightness;
      break;
    case COMMAND_TOGGLE:
      ledOn = (value > 0);
      break;
    case COMMAND_MODE:
      if (value >= MODE_MIN && value <= MODE_MAX) {
        ledMode = value;
      }
      break;
    case COMMAND_HUE:
      globalHue = value;
      break;
    case COMMAND_TEMPERATURE:
      temperature = value;
      break;
    default:
      Serial.print("Unknown command received: 0x");
      Serial.println(command, HEX);
      break;
  }
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

  // Debug output
  Serial.println("Temperature: " + String(temperature));
  Serial.println("Brightness: " + String(brightness));

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
void effectRainbow() {
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

void effectHue() {
  static float hotspots_left[3] = {0.0, NUM_LEDS * 0.3, NUM_LEDS * 0.7};
  static float hotspots_right[3] = {NUM_LEDS * 0.15, NUM_LEDS * 0.45, NUM_LEDS * 0.85};
  static float speeds_left[3] = {0.02, 0.03, 0.025};
  static float speeds_right[3] = {0.018, 0.027, 0.022};
  static unsigned long lastUpdate = 0;
  static unsigned long lastSpeedChange = 0; // Track when we last changed speeds
  static unsigned long lastJump = 0;        // Track when hotspots last jumped
  
  const uint16_t HUE_RANGE = 1000;
  const unsigned long MIN_SPEED_CHANGE_INTERVAL = 2000; // Minimum 2 seconds between speed changes
  const unsigned long MIN_JUMP_INTERVAL = 5000;         // Minimum 5 seconds between jumps
  
  unsigned long now = millis();
  if (now - lastUpdate >= 10) { // Update frequency
    lastUpdate = now;
    
    // Update hotspot positions with slow movement, applying speed modifier
    for (int i = 0; i < 3; i++) {
      hotspots_left[i] += speeds_left[i] * speedModifier;
      hotspots_right[i] += speeds_right[i] * speedModifier;
      
      // Reset positions when they exceed strip length
      if (hotspots_left[i] >= NUM_LEDS) hotspots_left[i] -= NUM_LEDS;
      if (hotspots_right[i] >= NUM_LEDS) hotspots_right[i] -= NUM_LEDS;
    }
    
    // Handle speed changes less frequently
    if (now - lastSpeedChange > MIN_SPEED_CHANGE_INTERVAL) {
      if (random(1000) < 10) { // 1% chance when we check
        for (int i = 0; i < 3; i++) {
          // Keep speeds in a reasonable range
          speeds_left[i] = 0.01 + (random(15) / 1000.0);
          speeds_right[i] = 0.01 + (random(15) / 1000.0);
        }
        lastSpeedChange = now;
      }
    }
    
    // Clear all strips
    strip_r1.clear();
    strip_r2.clear();
    strip_l1.clear();
    strip_l2.clear();
    
    // Apply the lava effect to left strips
    for (int i = 0; i < NUM_LEDS; i++) {
      float intensity = 0.0;
      
      // Calculate influence from all hotspots
      for (int h = 0; h < 3; h++) {
        float distance = min(abs(i - hotspots_left[h]), NUM_LEDS - abs(i - hotspots_left[h])) / (float)(NUM_LEDS/4);
        // Stronger, more focused hotspots
        float hotspotInfluence = exp(-distance * distance * 2);
        intensity = max(intensity, hotspotInfluence);
      }
      
      // Apply a subtle variation to the hue
      uint16_t pixelHue = globalHue + (uint16_t)(intensity * HUE_RANGE);
      
      // Set colors with full saturation and slightly variable value for depth
      uint8_t value = 220 + (uint8_t)(intensity * 35);  // 220-255 range for value
      uint32_t color = strip_l1.ColorHSV(pixelHue, 255, value);
      strip_l1.setPixelColor(i, color);
      strip_l2.setPixelColor((i + STAGGER_OFFSET) % NUM_LEDS, color);
    }
    
    // Apply the lava effect to right strips (similar but with different hotspot positions)
    for (int i = 0; i < NUM_LEDS; i++) {
      float intensity = 0.0;
      
      for (int h = 0; h < 3; h++) {
        float distance = min(abs(i - hotspots_right[h]), NUM_LEDS - abs(i - hotspots_right[h])) / (float)(NUM_LEDS/4);
        float hotspotInfluence = exp(-distance * distance * 2);
        intensity = max(intensity, hotspotInfluence);
      }
      
      uint16_t pixelHue = globalHue + (uint16_t)(intensity * HUE_RANGE);
      uint8_t value = 220 + (uint8_t)(intensity * 35);
      uint32_t color = strip_r1.ColorHSV(pixelHue, 255, value);
      strip_r1.setPixelColor(i, color);
      strip_r2.setPixelColor((i + STAGGER_OFFSET) % NUM_LEDS, color);
    }

    // Set the first pixel on each left strip to black
    strip_l1.setPixelColor(0, 0);
    strip_l2.setPixelColor(0, 0);
    
    // Update all strips
    strip_l1.show();
    strip_l2.show();
    strip_r1.show();
    strip_r2.show();
  }
}

void setAllStripsToHue(uint16_t hue) {
  // Convert HSV to RGB color
  uint32_t color = strip_r1.ColorHSV(hue, 255, 255);
  
  // Fill all strips with the same color
  strip_r1.fill(color, 0, NUM_LEDS);
  strip_r2.fill(color, 0, NUM_LEDS);
  strip_l1.fill(color, 0, NUM_LEDS);
  strip_l2.fill(color, 0, NUM_LEDS);

  strip_l1.setPixelColor(0, 0);
  strip_l2.setPixelColor(0, 0);
  
  // Show the changes on all strips
  strip_r1.show();
  strip_r2.show();
  strip_l1.show();
  strip_l2.show();
}

void effectWhite() {
  // temperature: 0 (cool) to 65535 (warm)
  uint16_t maxDuty = (1 << PWM_RESOLUTION) - 1; // Maximum duty cycle for the set resolution
  uint16_t scaledBrightness = map(brightness, 0, 255, 0, maxDuty);
  
  uint16_t warmDuty = map(temperature, 0, 65535, 0, scaledBrightness);
  uint16_t coolDuty = map(temperature, 0, 65535, scaledBrightness, 0);

  setWhiteLEDs(warmDuty, coolDuty);
}

void loop() {
  ArduinoOTA.handle();
  effects[ledMode]();
}

