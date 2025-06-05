#include "ESPNowClient.h"

void (*ESPNowClient::userReceiveCallback)(const uint8_t *mac, const uint8_t *data, size_t len) = nullptr;
void (*ESPNowClient::userSendCallback)(const uint8_t *mac, esp_now_send_status_t status) = nullptr;
uint8_t ESPNowClient::peerAddr[6] = {0}; // Not used in receive-only mode

ESPNowClient::ESPNowClient() {}

bool ESPNowClient::begin() {
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW initialization failed!");
        return false;
    }
    esp_now_register_recv_cb(onReceive);
    Serial.println("ESP-NOW initialized in receive-only mode");
    return true;
}

void ESPNowClient::setReceiveCallback(void (*callback)(const uint8_t *mac, const uint8_t *data, size_t len)) {
    userReceiveCallback = callback;
}

void ESPNowClient::onReceive(const uint8_t *mac, const uint8_t *data, int len) {
    if (userReceiveCallback) {
        userReceiveCallback(mac, data, len);
    }
}

// These methods remain for compatibility but are simplified
bool ESPNowClient::setPeerAddress(const uint8_t *peer) {
    // Not necessary for receive-only mode, but kept for API compatibility
    return true;
}

bool ESPNowClient::sendMessage(const uint8_t *data, size_t len) {
    // Not used in receive-only mode
    return false;
}

void ESPNowClient::setSendCallback(void (*callback)(const uint8_t *mac, esp_now_send_status_t status)) {
    // Not used in receive-only mode
    userSendCallback = callback;
}

void ESPNowClient::onSend(const uint8_t *mac, esp_now_send_status_t status) {
    // Not used in receive-only mode
    if (userSendCallback) {
        userSendCallback(mac, status);
    }
}
