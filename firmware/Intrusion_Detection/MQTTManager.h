#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <WiFi.h>

//Core MQTT library
#include <PubSubClient.h>


class MQTTManager {
private:
  WiFiClient espClient; // Creates RAW TCP cocket conenction
  PubSubClient client;  // Actual MQTT instance
  
  const char* broker; // MQTT broker domain 
  uint16_t port;  // 1833 for unencrypted MQTT
  const char* user; // Username
  const char* pass; // Key

  // State variables for non blocking timing
  unsigned long lastReconnectAttempt = 0;
  const unsigned long reconnectInterval = 5000; // Retry connection every 5s non-blockingly

  bool reconnect() {

    // Creates a unique client id string; Broker kicks out existing client if another device conencts with the same ID
    String clientId = "ESP32S3-Client-";
    clientId += String(random(0xffff), HEX);

    // Sends the connect packet to the broker along with the credentials
    if (client.connect(clientId.c_str(), user, pass)) {
      Serial.println("[MQTT] Connected to Broker!");
      return true;
    } else {
      Serial.print("[MQTT] Connection failed, rc=");

      // Prints the exact numerical return if connection fails: -2 Server unreachable; -4 Bad credentials
      Serial.println(client.state());
      return false;
    }
  }

public:
  MQTTManager() : client(espClient) {} // Binds the PubSubClinet wrapper directly to the underlying TCP socket

  // Configures the object with credentials
  void begin(const char* brokerUrl, uint16_t brokerPort, const char* username, const char* password) {
    broker = brokerUrl;
    port = brokerPort;
    user = username;
    pass = password;

    client.setServer(broker, port);
  }

  // Non-blocking loop: manages reconnects and keeps MQTT active
  void update() {
    if (!client.connected()) {
      unsigned long now = millis();
      if (now - lastReconnectAttempt >= reconnectInterval) {
        lastReconnectAttempt = now;
        Serial.println("[MQTT] Attempting non-blocking reconnect...");
        reconnect();
      }
    } else {
      client.loop();
    }
  }

  // Publish payload to a specific topic
  bool publish(const char* topic, const char* payload) {
    if (client.connected()) {
      bool success = client.publish(topic, payload);
      if (success) {
        Serial.print("[MQTT] Published to ");
        Serial.print(topic);
        Serial.print(": ");
        Serial.println(payload);
      } else {
        Serial.println("[MQTT] Publish failed!");
      }
      return success;
    }
    Serial.println("[MQTT] Cannot publish: Not connected.");
    return false;
  }

  bool isConnected() {
    return client.connected();
  }
};

#endif