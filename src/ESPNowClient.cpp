#include "ESPNowClient.h"

void (*ESPNowClient::userReceiveCallback)(const uint8_t *mac, const uint8_t *data, size_t len) = nullptr;
void (*ESPNowClient::userSendCallback)(const uint8_t *mac, esp_now_send_status_t status) = nullptr;
uint8_t ESPNowClient::peerAddr[6] = {0}; // Default to 00:00:00:00:00:00

ESPNowClient::ESPNowClient() {}

bool ESPNowClient::begin() {
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW initialization failed!");
        return false;
    }
    esp_now_register_recv_cb(onReceive);
    esp_now_register_send_cb(onSend);
    return true;
}

bool ESPNowClient::setPeerAddress(const uint8_t *peer) {
    memcpy(peerAddr, peer, 6);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, peerAddr, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return false;
    }
    return true;
}

bool ESPNowClient::sendMessage(const uint8_t *data, size_t len) {
    return esp_now_send(peerAddr, data, len) == ESP_OK;
}

void ESPNowClient::setReceiveCallback(void (*callback)(const uint8_t *mac, const uint8_t *data, size_t len)) {
    userReceiveCallback = callback;
}

void ESPNowClient::setSendCallback(void (*callback)(const uint8_t *mac, esp_now_send_status_t status)) {
    userSendCallback = callback;
}

void ESPNowClient::onReceive(const uint8_t *mac, const uint8_t *data, int len) {
    if (userReceiveCallback) {
        userReceiveCallback(mac, data, len);
    }
}

void ESPNowClient::onSend(const uint8_t *mac, esp_now_send_status_t status) {
    if (userSendCallback) {
        userSendCallback(mac, status);
    }
}
