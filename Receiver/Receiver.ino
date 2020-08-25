/**
   ESPNOW - Basic communication - Slave
   Date: 26th September 2017
   Author: Arvind Ravulavaru <https://github.com/arvindr21>
   Purpose: ESPNow Communication between a Master ESP32 and a Slave ESP32
   Description: This sketch consists of the code for the Slave module.
   Resources: (A bit outdated)
   a. https://espressif.com/sites/default/files/documentation/esp-now_user_guide_en.pdf
   b. http://www.esploradores.com/practica-6-conexion-esp-now/

   << This Device Slave >>

   Flow: Master
   Step 1 : ESPNow Init on Master and set it in STA mode
   Step 2 : Start scanning for Slave ESP32 (we have added a prefix of `slave` to the SSID of slave for an easy setup)
   Step 3 : Once found, add Slave as peer
   Step 4 : Register for send callback
   Step 5 : Start Transmitting data from Master to Slave

   Flow: Slave
   Step 1 : ESPNow Init on Slave
   Step 2 : Update the SSID of Slave with a prefix of `slave`
   Step 3 : Set Slave in AP mode
   Step 4 : Register for receive callback and wait for data
   Step 5 : Once data arrives, print it in the serial monitor

   Note: Master and Slave have been defined to easily understand the setup.
         Based on the ESPNOW API, there is no concept of Master and Slave.
         Any devices can act as master or salve.
*/

#include <esp_now.h>
#include <WiFi.h>

#if defined(ARDUINO_ARCH_ESP32)
#include <ESP32Servo.h>
#else
#include <Servo.h>
#endif

#include <ESC.h>
ESC myESC (13, 1000, 2000, 500);         // ESC_Name (PIN, Minimum Value, Maximum Value, Arm Value)

Servo steeringServo;

#include "RC.h"

#define WIFICHANNEL 1

#define RCCHANNELS 8

#define RCDeviceName "Defender90"

long lastHeartbeat;
long heartbeatLength = 500;  // How long, in millis, before a loss of signal triggers fail safe mode


// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

// config AP SSID
void configDeviceAP() {
  const char *SSID = "ESPRC_" RCDeviceName;
  bool result = WiFi.softAP(SSID, "ESPRC_Password", WIFICHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
  }
}

void setup() {
  Serial.begin(115200);
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_AP);
  // configure device AP mode
  configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_recv_cb(OnDataRecv);
  steeringServo.attach(12);
  myESC.arm();                          // Send the Arm value
}

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  RCdataTY* RCdata = (RCdataTY*) data;
//  Serial.print("********************");
//  Serial.print("DATA LENGTH: ");
//  Serial.println(data_len);
//  if (data_len == 16) //RCdataSize
//  {
    for (int i=0;i<RCCHANNELS;i++) {  //RCdataSize
//      RCdata.Ch[i] = data[i];
      Serial.print("Channel: ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(RCdata->Ch[i]);
    }
    steeringServo.write(map(RCdata->Ch[0],0,255,0,180));
    int speedVal = map(RCdata->Ch[1], 128, 255, 1000, 2000);  // scale it to use it with the ESC (value between 1000 and 2000)
    myESC.speed(speedVal);                     // sets the ESC speed according to the scaled value

    //}
  lastHeartbeat = millis();

}

void loop() {
  // Chill
    unsigned long currentMillis = millis();
  if (currentMillis - lastHeartbeat > heartbeatLength) {
      Serial.println("MISSED HEARTBEAT");    
      myESC.speed(500);                     // sets the ESC speed according to the scaled value

      
  }
}
