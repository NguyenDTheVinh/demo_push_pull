
#include <FirebaseESP32.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <Adafruit_ADS1X15.h>
#include <DHT.h>
#include <LoRa.h>
#include <SPI.h>

#define DHTPIN 27 // output cảm biển vào chân D5 = I/O 14
#define DHTTYPE DHT11 // set type là DHT11

#define relay1 26 // GPIO 0 ~ D3 -- 31/5/2022
#define moisture 35 // GPIO 0 ~ D3 -- 31/5/2022
#define light 33 // GPIO 0 ~ D3 -- 31/5/2022

#define wifiLed 13

#define ss 5
#define rst 14
#define dio0 2

int count = 0;
// int stop;
int count_Json_error = 0;

/*-------LoRa variables-----*/
String Receive_Data = "";
String Send_Data = "";
// DynamicJsonBuffer jsonBuffer;
// DynamicJsonBuffer  jsonBuffer(200);
StaticJsonDocument<200> doc;

int auto2, L_relay;
int relay2 = 0;

unsigned long startMillis = 0;
/*-----------------------------------Prepare for Connection------------------------------------------*/
/*-------------------------------------Firebase connection-------------------------------------------*/
#define FIREBASE_HOST "https://smartgreen-e57d2-default-rtdb.firebaseio.com/" // Firebase Admin SDK
#define FIREBASE_AUTH "sQlrUYAJt4OQGh77Um7S5QWDKIPyCESnRLEZREQz" // Database secret key
FirebaseData firebaseData;
FirebaseJson json;

String data ="";
// String path = "Pump";
String path_A = "A"; // Firebase database reference
String path_B = "B"; // Firebase database reference

    float re_humi = 0;
    float re_temp = 0; 
    int re_mois = 0; 
    float re_light = 0;
    int re_auto = 0;
    int re_relay = 0;

    int pump1, pump2, percent_1, avg_ms;
    int goDatabase = 0;

    DHT dht(DHTPIN,DHTTYPE);
    float air_humi; // Humidity
    float air_tempC; // Temperature

    int ms = 0;
    int l = 0;

/*---------------------------------------------------------------------------------------------------*/

/*---------------------------------------WiFi connection---------------------------------------------*/
WiFiClient client;
WiFiServer server(80);

const char* ssid = "Hung Dien";
const char* password = "hungdien1972";
// const char* ssid = "Galaxy M5165AB";
// const char* password = "mouf7022";
/*---------------------------------------------------------------------------------------------------*/

/*---------------------Setup here-------------------------*/

void setup() {
  Serial.begin(9600);
  // WiFi.mode(WIFI_MODE_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  pinMode(wifiLed, OUTPUT);
  digitalWrite(wifiLed, HIGH);

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH); // Start connecting to Firebase database

  LoRa.setPins(ss, rst, dio0);    //setup LoRa transceiver module
  while (!LoRa.begin(433E6))     //433E6 - Asia, 866E6 - Europe, 915E6 - North America
  {
    Serial.println(".");
    delay(500);
  }
  LoRa.setSyncWord(0xA5);
  Serial.println("LoRa Initializing OK!");

  dht.begin(); // start DHT11

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH); // Start connecting to Firebase database
  Firebase.reconnectWiFi(true);
  Firebase.setReadTimeout(firebaseData, 1000);
  Firebase.setwriteSizeLimit(firebaseData, "tiny");
  pinMode(relay1, OUTPUT);
  digitalWrite(relay1, LOW);
}
/*--------------------------------------------------------*/

/*---------------------Loop here-------------------------*/
void loop() {
  check:
    int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet from the Slave: '");
    count = 0;
    // read packet
    while (LoRa.available()) {
      Receive_Data = LoRa.readString();
      Serial.print(Receive_Data);
    }
    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());

    // JsonObject& doc = jsonBuffer.parseObject(Receive_Data);
    // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, Receive_Data);
  // Test if parsing succeeds.
  if (error) {
    count_Json_error++;
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    if (count_Json_error == 10) {
      ESP.restart();
    }
    return;
  } 
    
    String str_humi = doc["humidity"];
    String str_temp = doc["temperature"];
    String str_mois = doc["moisture"];
    String str_light = doc["light"];
    String str_auto = doc["auto"];
    // String str_relay = doc["relay"];


     re_humi = str_humi.toFloat();
     re_temp = str_temp.toFloat();
     re_mois = str_mois.toInt();
     re_light = str_light.toFloat();
     re_auto = str_auto.toInt();
    //  re_relay = str_relay.toInt();

    // Firebase.setIntAsync(firebaseData, "B/Relay", re_relay);
    
    Firebase.setFloatAsync(firebaseData, "B/AirTempC", re_temp); 
    Firebase.setFloatAsync(firebaseData, "B/AirHumidity", re_humi);
    Firebase.setIntAsync(firebaseData, "B/Moisture", re_mois);
    Firebase.setFloatAsync(firebaseData, "B/Light", re_light);

    if (Firebase.getInt(firebaseData, "A/Pump") == true)
    { 
      pump1 = firebaseData.intData();
    }

    if (pump1 == 0) {
      digitalWrite(relay1, LOW); // Pump OFF
    }

    if (pump1 == 1) {
      digitalWrite(relay1, HIGH); // Pump ON
    }

    if (pump1 == 2 || pump1 == 3)
    {
      if (percent_1 < 27) { // soil moirure < 27%
        digitalWrite(relay1, HIGH); // Pump ON
        Firebase.setIntAsync(firebaseData, "A/Relay", digitalRead(relay1));
      }

      if (percent_1 > 50) { // soil moirure > 45%
        digitalWrite(relay1, LOW); // Pump OFF
        Firebase.setIntAsync(firebaseData, "A/Relay", digitalRead(relay1));
      }
    }
    goDatabase = 1;
  } 
  else {
    if (count == 20) {
      ESP.restart();
    }
  }

  if ((millis() - startMillis) > 600)
  {
    if (Firebase.getInt(firebaseData, "B/Pump") == true)
    { 
      pump2 = firebaseData.intData();
    }
    Serial.print("The Master sending packet: ");
    Send_Data = 
    "{\"pump\":\"" + String(pump2) + "\"}";
    // stop = 1;
    LoRa.beginPacket();   //Send LoRa packet to receiver
    LoRa.print(Send_Data);
    LoRa.endPacket();
    Serial.println(Send_Data);
    // if ()
    count++;
    startMillis = millis();
  }

if (goDatabase == 1) {
      air_humi = dht.readHumidity();
      air_tempC = dht.readTemperature();
      Firebase.setFloatAsync(firebaseData, "A/AirTempC", air_tempC);
      Firebase.setFloatAsync(firebaseData, "A/AirHumidity", air_humi);

      for (int i = 0; i <=9; i++) {
        ms += analogRead(moisture);
      }
      avg_ms = ms/10;
      ms = 0;
      
      percent_1 = map(avg_ms, 1590, 4095, 100, 0);
      // Serial.println(percent_1);
      Firebase.setIntAsync(firebaseData, "A/Moisture", percent_1);
      // l = analogRead(light);
      float volts = analogRead(light) * 3.3 / 4095.0;
      float amps = volts / 10000.0; // 10,000 Ohms
      float microamps = amps * 1000000;
      float lux = microamps * 2.0;
      int lux_2d = lux*100;
      lux = float(lux_2d)/100;
      // Serial.println(lux_2d);
      Firebase.setFloatAsync(firebaseData, "A/Light", lux);
    // }
    Serial.println("__________________________________________________________________________________________________________________");
    goDatabase = 0;
  }
}