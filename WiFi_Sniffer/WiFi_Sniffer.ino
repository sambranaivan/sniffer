
/*
  Copyright 2017 Andreas Spiess

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
  to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
  FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
  WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

  This software is based on the work of Ray Burnette: https://www.hackster.io/rayburne/esp8266-mini-sniff-f6b93a
*/

#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
// #include <credentials.h>
#include <set>
#include <string>
#include "./functions.h"
#include "./mqtt.h"

#define disable 0
#define enable  1
#define SENDTIME 30000//30 segundos
#define MAXDEVICES 100
#define JBUFFER 15+ (MAXDEVICES * 40)
#define PURGETIME 200000//200 segundos 3 minutos
#define MINRSSI -65

// uint8_t channel = 1;
unsigned int channel = 1;
int clients_known_count_old, aps_known_count_old;
unsigned long sendEntry, deleteEntry;
char jsonString[JBUFFER];


String device[MAXDEVICES];
int nbrDevices = 0;
int usedChannels[15];

#ifndef CREDENTIALS
// SERVIDOR MQTT
  //servidor LAN
  // const char *mqttServer = "192.168.1.2";
  //servidor Linsse

  
  //SSID WIFI
  
  //ivan local
  // #define mySSID "TOTOLINK N600RD"
  // #define myPASSWORD "sambrana50"

//ivan 4g
#define mySSID "iot"
#define myPASSWORD "12345678"


// #define mySSID "Fibertel WiFi027 2.4GHz"
// #define myPASSWORD "0043591365"
#endif

StaticJsonBuffer<JBUFFER>  jsonBuffer;

void setup() {
  Serial.begin(115200);
 

  Serial.println(F("CHIP ID: "));
  Serial.println(ESP.getChipId());
  pinMode(2, OUTPUT);
  pinMode(16, OUTPUT);
  

  wifi_set_opmode(STATION_MODE);            // Promiscuous works only with station mode
  wifi_set_channel(channel);
  wifi_promiscuous_enable(disable);
  wifi_set_promiscuous_rx_cb(promisc_cb);   // Set up promiscuous callback
  wifi_promiscuous_enable(enable);
  

}




void loop() {
  digitalWrite(2, LOW);
  digitalWrite(16, HIGH);
  channel = 1;
  boolean sendMQTT = false;
  wifi_set_channel(channel);
  while (true) {
    nothing_new++;                          // Array is not finite, check bounds and adjust if required
    if (nothing_new > 200) {                // monitor channel for 200 ms
      nothing_new = 0;
      channel++;
      if (channel == 15) break;             // Only scan channels 1 to 14
      wifi_set_channel(channel);
    }
    delay(1);  // critical processing timeslice for NONOS SDK! No delay(0) yield()

    if (clients_known_count > clients_known_count_old) {
      clients_known_count_old = clients_known_count;
      sendMQTT = true;
    }
    if (aps_known_count > aps_known_count_old) {
      aps_known_count_old = aps_known_count;
      sendMQTT = true;
    }
    if (millis() - sendEntry > SENDTIME) {
      sendEntry = millis();
      sendMQTT = true;
    }
  }
  purgeDevice();
  
  if (sendMQTT) {
    showDevices();
    sendDevices();
  }
  else{
    ///nothin new 
  }
}


void connectToWiFi() {
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println(ESP.getFreeHeap());
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(mySSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(mySSID, myPASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void purgeDevice() {
  for (int u = 0; u < clients_known_count; u++) {
    if ((millis() - clients_known[u].lastDiscoveredTime) > PURGETIME) {
      Serial.print("purge Client" );
      Serial.println(u);
      for (int i = u; i < clients_known_count; i++) memcpy(&clients_known[i], &clients_known[i + 1], sizeof(clients_known[i]));
      clients_known_count--;
      break;
    }
  }
  for (int u = 0; u < aps_known_count; u++) {
    if ((millis() - aps_known[u].lastDiscoveredTime) > PURGETIME) {
      Serial.print("purge Bacon" );
      Serial.println(u);
      for (int i = u; i < aps_known_count; i++) memcpy(&aps_known[i], &aps_known[i + 1], sizeof(aps_known[i]));
      aps_known_count--;
      break;
    }
  }
}


void showDevices() {
  Serial.println("");
  Serial.println("");
  Serial.println("-------------------Device DB-------------------");
  Serial.printf("%4d Devices + Clients.\n",aps_known_count + clients_known_count); // show count

  // show Beacons
  for (int u = 0; u < aps_known_count; u++) {
    Serial.printf( "%4d ",u); // Show beacon number
    Serial.print("B ");
    Serial.print(formatMac1(aps_known[u].bssid));
    Serial.print(" RSSI ");
    Serial.print(aps_known[u].rssi);
    Serial.print(" channel ");
    Serial.println(aps_known[u].channel);
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
     //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  }

  // show Clients
  for (int u = 0; u < clients_known_count; u++) {
    Serial.printf("%4d ",u); // Show client number
    Serial.print("C ");
    Serial.print(formatMac1(clients_known[u].station));
    Serial.print(" RSSI ");
    Serial.print(clients_known[u].rssi);
    Serial.print(" channel ");
    Serial.println(clients_known[u].channel);
     //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
     //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  }
}

void sendDevices() {
  String deviceMac;
  digitalWrite(2, HIGH);
  digitalWrite(16, LOW);
  // Setup MQTT
  wifi_promiscuous_enable(disable);
  connectToWiFi();
  client.setServer(mqttServer, 1883);
  while (!client.connected()) {
    Serial.println("Conectando a MQTT");
    Serial.println(mqttServer);

    if (client.connect("ESP32Client")) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.println(client.state());
    }
    yield();
  }

  // Purge json string
  jsonBuffer.clear();
  JsonObject& root = jsonBuffer.createObject();
  JsonArray& dispositivos = root.createNestedArray("devices");
 
 

 /*
  {[
    {"MAC":ff:ff:ff:ff:ff, "rssi":22, "chanel": 4}
  ]}

 */

  // add Beacons
  for (int u = 0; u < aps_known_count; u++) {
    deviceMac = formatMac1(aps_known[u].bssid);
  
    JsonObject& devObj = jsonBuffer.createObject();
    devObj["M"] = deviceMac;
    devObj["R"] = aps_known[u].rssi;
    devObj["C"] = aps_known[u].channel;
    dispositivos.add(devObj);
    // }
  }

  // Add Clients
  for (int u = 0; u < clients_known_count; u++) {
    deviceMac = formatMac1(clients_known[u].station);
    // if (clients_known[u].rssi > MINRSSI) { // filtro de minimo rrsi

     // mac.add(deviceMac);
      //rssi.add(clients_known[u].rssi);
    JsonObject& devObj = jsonBuffer.createObject();
    devObj["M"] = deviceMac;
    devObj["R"] = clients_known[u].rssi;
    devObj["C"] = clients_known[u].channel;
    dispositivos.add(devObj);    
    // }
  }

  Serial.println();
  Serial.printf("number of devices: %02d\n", dispositivos.size());
  root.prettyPrintTo(Serial);
  root.printTo(jsonString);
  //  Serial.println((jsonString));
  //  Serial.println(root.measureLength());
  String topic = "sniffer/";
  topic = topic + String(ESP.getChipId());
  char topico[20];
  topic.toCharArray(topico,20);

  

  if (client.publish(topico, jsonString) == 1)
   {
     Serial.println("Successfully published on " + topic);
   }

  else {
    Serial.println();
    Serial.println(" No Publicado Limite de Paquete Excedido");
    Serial.println();
  }

  // ///send head
  // String topic = "snifferHeap/";
  // topic = topic + String(ESP.getChipId());
  // char topico[30];
  // topic.toCharArray(topico, 30);
  // client.publish(topico, jsonString)

  // ///end send headp

  client.loop();
  client.disconnect();
  delay(100);
  wifi_promiscuous_enable(enable);
  sendEntry = millis();
}
