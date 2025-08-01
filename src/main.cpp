#include <Arduino.h>
#include "config.h"  // Using configuration constants defined in config.h
#include "ESPNowClient.h"
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>

ESPNowClient espNow;

// Temperature is the proportion of warm white to cool white
// 0 is all cool white, 255 is all warm white
uint8_t temperature = 127;
uint8_t brightness = 255; // Using uint8_t with range 0-255
uint8_t globalHue = 0;
bool ledOn = true;
float speedModifier = 3.0;
int ledMode = 0; // 0: Rainbow, 1: Hue, 2: White

// Global RGB clearing state
bool rgbStripsCleared = false;

// Button state tracking
bool buttonPressed = false;
unsigned long lastButtonPress = 0;  // For debouncing


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

Adafruit_NeoPixel strip_l1(NUM_LEDS, PIN_DATA1, NEO_RGB + NEO_KHZ800);
Adafruit_NeoPixel strip_l2(NUM_LEDS, PIN_DATA2, NEO_RGB + NEO_KHZ800);
Adafruit_NeoPixel strip_r1(NUM_LEDS, PIN_DATA3, NEO_RGB + NEO_KHZ800);
Adafruit_NeoPixel strip_r2(NUM_LEDS, PIN_DATA4, NEO_RGB + NEO_KHZ800);

// Updates both LED channels based on ledOn and brightness
void setWhiteLEDs(uint16_t warmDuty, uint16_t coolDuty) {
  uint16_t warmDutyCalcd = ledOn ? warmDuty : 0;
  uint16_t coolDutyCalcd = ledOn ? coolDuty : 0;
  
  ledcWrite(PWM_CHANNEL_0, coolDutyCalcd);
  ledcWrite(PWM_CHANNEL_1, warmDutyCalcd);
}

// Button handler for toggling LED state
void checkButton() {
  bool currentState = digitalRead(BUTTON_PIN);
  unsigned long now = millis();
  
  // Check if button is pressed (LOW when pressed because of pull-up)
  if (!currentState && !buttonPressed && (now - lastButtonPress > BUTTON_DEBOUNCE_TIME)) {
    buttonPressed = true;
    lastButtonPress = now;
      // Cycle through modes
    ledMode += 1;
    if (ledMode > MODE_MAX) {
      ledMode = MODE_MIN; // Reset to first mode
    }
  } else if (currentState && buttonPressed) {
    // Button released
    buttonPressed = false;
  }
}

void onReceive(const uint8_t *mac, const uint8_t *data, size_t len) {
  // Debug output for ESP-NOW message reception
  Serial.print("ESP-NOW message received from: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(mac[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.print(" Length: ");
  Serial.print(len);
  Serial.print(" Data[0]: ");
  Serial.println(data[0], HEX);
  
  if (len < 2) return; // Need at least command and 1 byte of state
  
  uint8_t command = data[0];
  uint8_t value8bit = data[1];  // Always use 8-bit value from remote
  
  Serial.print("Processed command: 0x");
  Serial.print(command, HEX);
  Serial.print(" Value: ");
  Serial.println(value8bit);
  
  switch(command) {
    case COMMAND_BRIGHTNESS:
      brightness = value8bit; // Use 8-bit value (0-255)
      {
        // Also update the NeoPixel brightness immediately
        strip_r1.setBrightness(brightness);
        strip_r2.setBrightness(brightness);
        strip_l1.setBrightness(brightness);
        strip_l2.setBrightness(brightness);
      }
      break;    case COMMAND_TOGGLE:
      if (value8bit > 0) {
        // Turn on LEDs
        ledOn = true;
      } else {
        // Turn off LEDs
        ledOn = false;
      }
      break;    case COMMAND_MODE:
      // Remote sends 0-2 for valid modes, map directly
      if (value8bit >= 0 && value8bit <= 2) {
        ledMode = value8bit;
      }
      break;
    case COMMAND_HUE:
      globalHue = value8bit;
      break;
    case COMMAND_TEMPERATURE:
      temperature = value8bit;
      break;
    default:
      Serial.print("Unknown command received: 0x");
      Serial.println(command, HEX);
      break;
  }
}

// Show a specific boot stage indicator on the LED strips
void showBootIndicator(int stage) {
  // All strips should be initialized before calling this
  uint32_t colorRed = strip_r1.Color(255, 0, 0);      // Red
  uint32_t colorGreen = strip_r1.Color(0, 255, 0);    // Green
  uint32_t colorBlue = strip_r1.Color(0, 0, 255);     // Blue
  uint32_t colorYellow = strip_r1.Color(255, 255, 0); // Yellow
  uint32_t colorPurple = strip_r1.Color(128, 0, 255); // Purple
  uint32_t colorCyan = strip_r1.Color(0, 255, 255);   // Cyan
  uint32_t colorWhite = strip_r1.Color(255, 255, 255); // White
  
  // Ensure white LED strips (PWM controlled) are off during boot
  ledcWrite(PWM_CHANNEL_0, 0);
  ledcWrite(PWM_CHANNEL_1, 0);
  
  // Clear all strips first
  strip_r1.clear();
  strip_r2.clear();
  strip_l1.clear();
  strip_l2.clear();
  
  int numLedsToLight = 5; // Number of LEDs to light in each stage
  uint32_t stageColor;
  
  // Select color based on boot stage
  switch(stage) {
    case 1: // Initial boot
      stageColor = colorRed;
      break;
    case 2: // After ESP-NOW init
      stageColor = colorGreen;
      break;
    case 3: // After LEDs configured
      stageColor = colorPurple;
      break;
    case 4: // Ready to start main loop
      stageColor = colorWhite;
      break;
    default:
      stageColor = colorCyan;
  }
  
  // Light a section of LEDs on each strip based on the boot stage
  // Each stage lights up different LEDs so it's visually distinct
  int startPos = (stage - 1) * numLedsToLight % (NUM_LEDS - numLedsToLight);
  
  for (int i = 0; i < numLedsToLight; i++) {
    // Set different patterns for different strips to make it more visually distinctive
    strip_r1.setPixelColor(startPos + i, stageColor);
    strip_r2.setPixelColor((startPos + i + 2) % NUM_LEDS, stageColor);
    strip_l1.setPixelColor((startPos + i + 4) % NUM_LEDS, stageColor);
    strip_l2.setPixelColor((startPos + i + 6) % NUM_LEDS, stageColor);
  }
  
  // Show all strips
  strip_r1.show();
  strip_r2.show();
  strip_l1.show();
  strip_l2.show();
  
  delay(100); // Short delay to ensure the LEDs update
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 LED Controller Starting...");
  
  // Configure boot button pin with internal pull-up
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.println("Boot button configured on GPIO" + String(BUTTON_PIN));
    // Initialize LED strips early so we can use them for boot indicators
  strip_r1.begin();  
  strip_r2.begin();
  strip_l1.begin();
  strip_l2.begin();
  
  // Set brightness directly since we're now using 8-bit values (0-255)
  strip_r1.setBrightness(brightness);
  strip_r2.setBrightness(brightness);
  strip_l1.setBrightness(brightness);
  strip_l2.setBrightness(brightness);
  
  // Stage 1: Initial boot indicator
  showBootIndicator(1);
  
  // Initialize ESP-NOW first in receive-only mode
  WiFi.mode(WIFI_STA);  // Set WiFi to station mode for ESP-NOW
  WiFi.disconnect();    // Disconnect from any WiFi networks
  
  if (!espNow.begin()) {
    Serial.println("Failed to initialize ESP-NOW client");
  } else {
    Serial.println("ESP-NOW initialized successfully in receive-only mode");
    // Display MAC address for easy identification
    Serial.print("My MAC Address: ");
    Serial.println(WiFi.macAddress());
    
    // Stage 2: ESP-NOW initialized indicator
    showBootIndicator(2);
  }
  
  espNow.setReceiveCallback(onReceive);
  
  // LED strip setup
  ledcSetup(PWM_CHANNEL_0, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_1, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(PIN_PWM1, PWM_CHANNEL_0);
  ledcAttachPin(PIN_PWM2, PWM_CHANNEL_1);

  // Debug output
  Serial.println("Temperature: " + String(temperature));
  Serial.println("Brightness: " + String(brightness));
  
  // Stage 3: LEDs configured
  showBootIndicator(3);
  
  // Stage 4: Boot complete - ready to run main loop
  showBootIndicator(4);
  
  // Clear all strips to prepare for the main loop functionality
  strip_r1.clear();
  strip_r2.clear();
  strip_l1.clear();
  strip_l2.clear();
  strip_r1.show();
  strip_r2.show();
  strip_l1.show();
  strip_l2.show();
  // Also make sure the white LEDs start in the correct state
  if (ledMode == 2) {
    // Initialize white LEDs if in white mode
    uint16_t maxDuty = (1 << PWM_RESOLUTION) - 1; // 4095 for 12-bit resolution
    uint16_t scaledBrightness = map(brightness, BRIGHTNESS_MIN, BRIGHTNESS_MAX, 0, maxDuty);
    uint16_t warmDuty = map(temperature, TEMPERATURE_MIN, TEMPERATURE_MAX, 0, scaledBrightness);
    uint16_t coolDuty = map(temperature, TEMPERATURE_MIN, TEMPERATURE_MAX, scaledBrightness, 0);
    setWhiteLEDs(warmDuty, coolDuty);
  }
}

// New function: updateRainbowEffect() performs the rainbow animation using variable intervals
void effectRainbow() {
  // Turn off white LEDs in this mode
  static bool whiteLedsOff = false;
  static bool cleared = false;
  static int lastMode = -1;
  
  // Reset flags when mode changes
  if (lastMode != ledMode) {
    whiteLedsOff = false;
    lastMode = ledMode;
  }
  
  if (!whiteLedsOff) {
    setWhiteLEDs(0, 0);
    whiteLedsOff = true;
  }
  
  // If LEDs are off, clear all strips and return
  if (!ledOn) {
    if (!cleared) {
      strip_r1.clear(); strip_r1.show();
      strip_r2.clear(); strip_r2.show();
      strip_l1.clear(); strip_l1.show();
      strip_l2.clear(); strip_l2.show();
      cleared = true;
    }
    return;
  }
  
  // Reset cleared flag when LEDs are on
  cleared = false;
  
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
  // Turn off white LEDs in this mode
  static bool whiteLedsOff = false;
  static bool cleared = false;
  static int lastMode = -1;
  
  // Reset flags when mode changes
  if (lastMode != ledMode) {
    whiteLedsOff = false;
    lastMode = ledMode;
  }
  
  if (!whiteLedsOff) {
    setWhiteLEDs(0, 0);
    whiteLedsOff = true;
  }
  
  // If LEDs are off, clear all strips and return
  if (!ledOn) {
    if (!cleared) {
      strip_r1.clear(); strip_r1.show();
      strip_r2.clear(); strip_r2.show();
      strip_l1.clear(); strip_l1.show();
      strip_l2.clear(); strip_l2.show();
      cleared = true;
    }
    return;
  }
  
  // Reset cleared flag when LEDs are on
  cleared = false;
  
  static float hotspots_left[3] = {0.0, NUM_LEDS * 0.3, NUM_LEDS * 0.7};
  static float hotspots_right[3] = {NUM_LEDS * 0.15, NUM_LEDS * 0.45, NUM_LEDS * 0.85};
  static float speeds_left[3] = {0.02, 0.03, 0.025};
  static float speeds_right[3] = {0.018, 0.027, 0.022};
  static unsigned long lastUpdate = 0;
  static unsigned long lastSpeedChange = 0; // Track when we last changed speeds
  static unsigned long lastJump = 0;        // Track when hotspots last jumped
  
  const uint16_t HUE_RANGE = 3000; // Increased range for more hue variation with 8-bit input
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
      // Scale 8-bit globalHue (0-255) to 16-bit range (0-65535) for ColorHSV
      uint16_t baseHue = (uint16_t)globalHue << 8; // Convert 8-bit to 16-bit
      uint16_t pixelHue = baseHue + (uint16_t)(intensity * HUE_RANGE);
      
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
      
      uint16_t baseHue = (uint16_t)globalHue << 8; // Convert 8-bit to 16-bit
      uint16_t pixelHue = baseHue + (uint16_t)(intensity * HUE_RANGE);
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



void setAllStripsToHue(uint8_t hue) {
  // Convert 8-bit hue (0-255) to 16-bit (0-65535) for ColorHSV
  uint16_t hue16bit = (uint16_t)hue << 8;
  uint32_t color = strip_r1.ColorHSV(hue16bit, 255, 255);
  
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



void loop() {
  static int lastGlobalMode = -1;
  
  checkButton();  // Check if the boot button was pressed
    // Check if mode changed and turn off white LEDs if switching away from white mode
  if (lastGlobalMode != ledMode) {
    if (lastGlobalMode == 2 && ledMode != 2) {
      // Switching away from white mode - turn off white LEDs immediately
      ledcWrite(PWM_CHANNEL_0, 0);
      ledcWrite(PWM_CHANNEL_1, 0);
    }
    // Reset global RGB clearing flag when mode changes
    rgbStripsCleared = false;
    lastGlobalMode = ledMode;
  }
  
  // Only call effect function if mode is valid
  if (ledOn && ledMode >= 0 && ledMode <= 2) {
    effects[ledMode]();
  } else {
    // If LEDs are off, ensure all strips are cleared
    ledcWrite(PWM_CHANNEL_0, 0);
    ledcWrite(PWM_CHANNEL_1, 0);
    strip_r1.clear(); strip_r1.show();
    strip_r2.clear(); strip_r2.show();
    strip_l1.clear(); strip_l1.show();
    strip_l2.clear(); strip_l2.show();
  }
}

void effectWhite() {
  // Always clear RGB strips when entering white mode
  if (!rgbStripsCleared) {
    strip_r1.clear(); strip_r1.show();
    strip_r2.clear(); strip_r2.show();
    strip_l1.clear(); strip_l1.show();
    strip_l2.clear(); strip_l2.show();
    rgbStripsCleared = true;
  }
  
  // Handle white LEDs based on ledOn state
  if (ledOn) {
    // Calculate and set white LED values using specified code
    uint16_t maxDuty = (1 << PWM_RESOLUTION) - 1;
    uint16_t scaledBrightness = map(brightness, BRIGHTNESS_MIN, BRIGHTNESS_MAX, 0, maxDuty);
    uint16_t warmDuty = map(temperature, TEMPERATURE_MIN, TEMPERATURE_MAX, 0, scaledBrightness);
    uint16_t coolDuty = map(temperature, TEMPERATURE_MIN, TEMPERATURE_MAX, scaledBrightness, 0);
    
    // Directly write to PWM channels
    ledcWrite(PWM_CHANNEL_0, coolDuty);
    ledcWrite(PWM_CHANNEL_1, warmDuty);
  } else {
    // Turn off white LEDs when ledOn is false
    ledcWrite(PWM_CHANNEL_0, 0);
    ledcWrite(PWM_CHANNEL_1, 0);
  }
}

