#include <Arduino.h>
#include <ArduinoOTA.h>
#include "DHTesp.h"
#include "Ticker.h"
#include <TroykaMQ.h>

#define PRINT_DEBUG_MESSAGES

#define MODULE_NAME "esp8266-climate-station"
#define DHT_PIN D2
#define PIN_MQ135  A0
#define PIN_MQ135_CALIBRATION  45
#define SENDING_INTERVAL 30*1000

void initWiFi();
void initOTA();
void collectData();
void sendData();

const char* server = "api.thingspeak.com";
WiFiClient  client;
DHTesp dht;
MQ135 mq135(PIN_MQ135);
Ticker sendTimer(collectData, SENDING_INTERVAL);
String data;
bool hasData = false;

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("[" + String(MODULE_NAME) + "] Start setup");
  initWiFi();
  initOTA();

  dht.setup(DHT_PIN, DHTesp::DHT22);
  mq135.calibrate(PIN_MQ135_CALIBRATION);

  sendTimer.start();
}

void loop() {
  ArduinoOTA.handle();
  yield();
  sendTimer.update();
  sendData();
}

void collectData(){
  data = String();
  float temperature = dht.getTemperature();
  if(temperature != NAN){
    Serial.print("\tTemperature: " + String(temperature));
    data += "&field1=" + String(temperature);
  }
  float humidity = dht.getHumidity();
  if(humidity != NAN){
    Serial.print("\tHumidity: " + String(humidity));
    data += "&field2=" +  String(humidity);
  }
  unsigned long CO2 = mq135.readCO2();
  if(CO2 != NAN){
    Serial.print("\CO2: " + String(CO2));
    data += "&field3=" +  String(CO2);
  }
  unsigned long correctedCO2 = mq135.readCorrectedCO2(temperature, humidity);
  if(temperature != NAN && humidity != NAN && correctedCO2 != NAN){
    Serial.print("\Cor CO2: " + String(correctedCO2));
    data += "&field4=" +  String(correctedCO2);
  }
  Serial.println();
  hasData = true;
}

void sendData(){
  if(hasData){
    if (!client.connect(server, 80)) {
      Serial.println("connection failed");
      return;
    }
    String Link="GET /update?api_key="+String(API_KEY);  //Requeste webpage  
    Link = Link + data;
    Link = Link + " HTTP/1.1\r\n" + "Host: " + server + "\r\n" + "Connection: close\r\n\r\n";                
    client.print(Link);

    //---------------------------------------------------------------------
    //Wait for server to respond with timeout of 5 seconds
    int timeout=0;
    while((!client.available()) && (timeout < 500))     //Wait 5 seconds for data
    {
      delay(10);  //Use this with time out
      timeout++;
      yield();
    }
  
    //---------------------------------------------------------------------
    //If data is available before time out read it.
    if(timeout < 500)
    {
        while(client.available()){
            Serial.println(client.readString()); //Response from ThingSpeak       
        }
    }
    else
      Serial.println("Request timeout..");
    hasData = false;
  }
}

void initWiFi(){
  WiFi.mode(WIFI_STA);
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);
  Serial.print("ssid: ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(WiFi.status());
  }
  
  Serial.print("\nIP address: ");
  Serial.println(WiFi.localIP());
}

void initOTA(){
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(MODULE_NAME);
  ArduinoOTA.onStart([]() {
    Serial.println("Start OTA");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("End OTA");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();
}
