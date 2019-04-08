#include <Arduino.h>
#include <ArduinoOTA.h>
#include <DHTesp.h>
#include <TroykaMQ.h>
#include "sntp.h"

#define PRINT_DEBUG_MESSAGES

// Parameters
#define MODULE_NAME "esp8266-climate-station"

#define DHT_PIN D2
#define DHT_SAMPLE_TIMES 5
#define DHT_SAMPLE_INTERVAL 20 // ms

#define MQ135_PIN A0
#define MQ135_PIN_CALIBRATION 45

WiFiClient client;

// Sensors
DHTesp dht;
MQ135 mq135{ MQ135_PIN };

time_t sampling_time;

TempAndHumidity dht_data;
int humidity_samples;
int temperature_samples;

float co2;

void initOTA();
void collectData();
void sendData();

void initWiFi()
{
	Serial.printf("Init WiFi.\n");
	Serial.printf("SSID: %s\n", WIFI_SSID);

	WiFi.mode(WIFI_STA);
	WiFi.setAutoConnect(true);
	WiFi.setAutoReconnect(true);
	WiFi.begin(WIFI_SSID, WIFI_PASS);

	while (WiFi.status() != WL_CONNECTED)
	{
		delay(1000);
		Serial.printf("WiFi status: %d\n", WiFi.status());
	}

	const auto& ip = WiFi.localIP();
	Serial.printf("WiFi local IP address: %s\n", ip.toString().c_str());
}

void setup() {
	Serial.begin(115200);
	Serial.printf("\nStart setup \"%s\"\n", MODULE_NAME);

	dht.setup(DHT_PIN, DHTesp::DHT22);

	mq135.calibrate(MQ135_PIN_CALIBRATION);
	Serial.printf("MQ135:\nRo = %e\n", mq135.getRo());

	initWiFi();
	initOTA();

	configTime(0, 0, "time.google.com", "ru.pool.ntp.org");
	int last = 0;

	while(sntp_get_current_timestamp() < 1500000000)
	{
		if((millis() - last) > 1000)
		{
			last = millis();
			Serial.println(sntp_get_current_timestamp());
			Serial.println("Waiting for sntp.");
		}

		yield();
	}
}

void loop()
{
	ArduinoOTA.handle();
	yield();
	collectData();
	yield();
	sendData();
	delay(1000);
}

void collectData()
{
	humidity_samples = 0;
	temperature_samples = 0;
	dht_data = {};
	sampling_time = sntp_get_current_timestamp();

	for (int i = 0; i < DHT_SAMPLE_TIMES; ++i)
	{
		const auto& dht_sample = dht.getTempAndHumidity();

		if (dht_data.temperature != NAN)
		{
			Serial.printf("Temperature sample: %e\n", dht_sample.temperature);
			dht_data.temperature += dht_sample.temperature;
			++temperature_samples;
		}
		else
		{
			Serial.println("Temperature sample: NAN");
		}

		if (dht_data.humidity != NAN)
		{
			Serial.printf("Humidity sample: %e\n", dht_sample.humidity);
			dht_data.humidity += dht_sample.humidity;
			++humidity_samples;
		}
		else
		{
			Serial.println("Humidity sample: NAN");
		}

		delay(DHT_SAMPLE_INTERVAL);
	}

	if (humidity_samples >= 1)
	{
		dht_data.humidity /= humidity_samples;
		Serial.printf("Humidity: %e\n", dht_data.humidity);
	}
	else
	{
		Serial.println("Humidity: NAN");
	}

	if (temperature_samples >= 1)
	{
		dht_data.temperature /= temperature_samples;
		Serial.printf("Temperature: %e\n", dht_data.temperature);
	}
	else
	{
		Serial.println("Temperature: NAN");
	}

	co2 = NAN;

	if (temperature_samples > 0 && humidity_samples > 0)
	{
		// TODO (k.german): unsigned long? float?
		co2 = mq135.readCorrectedCO2(dht_data.temperature, dht_data.humidity);

		if (co2 != NAN)
		{
			Serial.println("CO2 has been corrected.");
		}
	}

	if (co2 == NAN)
	{
		// TODO (k.german): unsigned long? float?
		co2 = mq135.readCO2();
	}

	if (co2 == NAN)
	{
		Serial.println("CO2: NAN");
	}
	else
	{
		Serial.printf("CO2: %e\n", co2);
	}
}

void sendData()
{
	// TODO (k.german): printf to buffer?

	String time_str{ sampling_time };
	String data{};

	if (temperature_samples > 0)
	{
		data += GRAPHITE_PREFIX "t.avg " + String(dht_data.temperature) + ' ' + time_str + '\n';
	}

	if (humidity_samples > 0)
	{
		data += GRAPHITE_PREFIX "h.avg " + String(dht_data.humidity) + ' ' + time_str + '\n';
	}

	if (co2 != NAN)
	{
		data += GRAPHITE_PREFIX "co2.avg " + String(co2) + ' ' + time_str + '\n';
	}

	Serial.println("*** GRAPHITE PACKAGE BEGIN ***");
	Serial.print(data);
	Serial.println("*** GRAPHITE PACKAGE END ***");

	yield();

	if (!client.connect(GRAPHITE_HOST, GRAPHITE_PORT))
	{
		Serial.println("Connection failed.");
		return;
	}

	if (!client.connected())
	{
		return;
	}

	Serial.println("Sending data to graphite " GRAPHITE_HOST ".");
	client.println(data);
	// TODO (k.german): Ensure whether packet reaches its destination point or not.
	// Wait for client.available()?
	client.stop();
}

void initOTA(){
	// Port defaults to 8266
	// ArduinoOTA.setPort(8266);

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
