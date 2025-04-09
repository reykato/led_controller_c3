#ifndef CONFIG_H
#define CONFIG_H

// WiFi Configuration
#define WIFI_SSID     "new-kids-iot"
#define WIFI_PASSWORD "tangle4-tip-dun"

// PWM Configuration
#define PWM_FREQ       800   // frequency in Hz
#define PWM_CHANNEL_0  0
#define PWM_CHANNEL_1  1
#define PWM_RESOLUTION 8       // 8-bit resolution (0-255)

// LED Pins (mirrored)
#define PIN_PWM1 20
#define PIN_PWM2 21
#define PIN_DATA1 5
#define PIN_DATA2 6
#define PIN_DATA3 7
#define PIN_DATA4 8

// Command Constants
#define COMMAND_RESET 0x01
#define COMMAND_TOGGLE 0x02
#define COMMAND_MODE 0x03
#define COMMAND_TEMPERATURE 0x04
#define COMMAND_BRIGHTNESS 0x05
#define COMMAND_HUE 0x06

// BLE Mode and Brightness Value Range
#define MODE_MIN   0
#define MODE_MAX   255
#define BRIGHT_MIN 0
#define BRIGHT_MAX 255

// Effect Constants
#define RAINBOW_INTERVAL_HIGH 23
#define RAINBOW_INTERVAL_LOW  4

#endif
