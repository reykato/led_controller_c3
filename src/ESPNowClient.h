#ifndef ESP_NOW_CLIENT_H
#define ESP_NOW_CLIENT_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "config.h"

class ESPNowClient {
public:
    ESPNowClient();

    bool begin();
    bool setPeerAddress(const uint8_t *peerAddr);
    bool sendMessage(const uint8_t *data, size_t len);
    void setReceiveCallback(void (*callback)(const uint8_t *mac, const uint8_t *data, size_t len));
    void setSendCallback(void (*callback)(const uint8_t *mac, esp_now_send_status_t status));

private:
    static void onReceive(const uint8_t *mac, const uint8_t *data, int len);
    static void onSend(const uint8_t *mac, esp_now_send_status_t status);

    static void (*userReceiveCallback)(const uint8_t *mac, const uint8_t *data, size_t len);
    static void (*userSendCallback)(const uint8_t *mac, esp_now_send_status_t status);
    static uint8_t peerAddr[6];  // Store peer address
};

#endif // ESP_NOW_CLIENT_H
