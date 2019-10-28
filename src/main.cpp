#include <Arduino.h>
#include <ArduinoOTA.h>
#include "sntp.h"
#include <Sensors/BME280Sensor.h>
#include <Sensors/MHZ19BSensor.h>
#include <Sensors/DHT22Sensor.h>
#include <Sensors/MQ135Sensor.h>

// Parameters
#define SEND_INTERVAL 20 // in seconds

// Correction
#define TEMP_CORRECTION 0
#define HUMIDITY_CORRETION 0
#define PRESSURE_CORRECTION 0
#define CO2_CORRECTION 0

#define BME280_SCL_PIN D1 // also GPIO5/SCL
#define BME280_SDA_PIN D2 // also GPIO4/SDA

// NOTE (k.german): SDD3 used in QIO flashing mode. Therefore, use DIO mode
// instead to free this pin. It can make troubles on some NodeMCU's devkits.
#define DHT_OUT_PIN 10 // SDD3/GPIO10
#define DHT_SAMPLE_TIMES 5
#define DHT_SAMPLE_INTERVAL 20 // ms

#define Z19B_PWM_PIN D5 // also GPIO14/SCLK
#define Z19B_RX_PIN D6  // also GPIO12/MISO
#define Z19B_TX_PIN D7  // also GPIO13/MOSI/RXD2

WiFiClient client{};
time_t sampling_time{};

BME280Sensor bme280_sensor{};
MHZ19BSensor mhz19b_sensor{ Z19B_RX_PIN, Z19B_TX_PIN, Z19B_PWM_PIN };
DHT22Sensor dht22_sensor{ DHT_OUT_PIN, DHT_SAMPLE_TIMES, DHT_SAMPLE_INTERVAL };

void initOTA();
void collectData();
void initWiFi();
void sendData();
void initNTP();


void setup()
{
	Serial.begin(115200);
	Serial.printf("\nStart setup \"%s\"\n", MODULE_NAME);

	mhz19b_sensor.setup();
	bme280_sensor.setup();
	dht22_sensor.setup();

	initWiFi();
	initOTA();
	initNTP();
}

static int last = 0;
void loop()
{
	ArduinoOTA.handle();
	yield();	
	unsigned long now = millis();
	if ((now - last) > SEND_INTERVAL * 1000) {
		last = now;
		collectData();
		int waitAfterCollect = millis();
  		while((millis() - waitAfterCollect) < 1000){
			yield();
		}
		sendData();
	}	
}

void initWiFi()
{
	Serial.println("# Init WiFi.");
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

void initNTP()
{
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

void collectData()
{
	sampling_time = sntp_get_current_timestamp();

	dht22_sensor.measure();
	bme280_sensor.measure();

	// FIXME (k.german): Choose one way. Or use one way as a fallback.
	mhz19b_sensor.measure_by_uart();
	mhz19b_sensor.measure_by_pwm();
}

void sendData()
{
	// TODO (k.german): printf to buffer?
	String time_str{ sampling_time };
	String message{};

	const auto& dht22_data = dht22_sensor.data();

	if (!isnan(dht22_data.temperature))
	{
		message += GRAPHITE_PREFIX "dht22.t.avg " + String(dht22_data.temperature + TEMP_CORRECTION) + ' ' + time_str + '\n';
	}

	if (!isnan(dht22_data.humidity))
	{
		message += GRAPHITE_PREFIX "dht22.h.avg " + String(dht22_data.humidity + HUMIDITY_CORRETION) + ' ' + time_str + '\n';
	}

	// FIXME: Check if value is unavailable?
	auto co2 = mhz19b_sensor.data().co2_uart;
	if(!isnan(co2) != NAN)
		message += GRAPHITE_PREFIX "mhz19b.co2.uart.avg " + String(co2 + CO2_CORRECTION) + ' ' + time_str + '\n';
	co2 = mhz19b_sensor.data().co2_pwm;
	if(!isnan(co2) != NAN)
		message += GRAPHITE_PREFIX "mhz19b.co2.pwm.avg " + String(co2 + CO2_CORRECTION) + ' ' + time_str + '\n';

	const auto& bme280_data = bme280_sensor.data();

	if (!isnan(bme280_data.temperature))
	{
		message += GRAPHITE_PREFIX "bme280.t.avg " + String(bme280_data.temperature + TEMP_CORRECTION) + ' ' + time_str + '\n';
	}

	if (!isnan(bme280_data.humidity))
	{
		message += GRAPHITE_PREFIX "bme280.h.avg " + String(bme280_data.humidity + HUMIDITY_CORRETION) + ' ' + time_str + '\n';
	}

	if (!isnan(bme280_data.pressure))
	{
		message += GRAPHITE_PREFIX "bme280.p.avg " + String(bme280_data.pressure + PRESSURE_CORRECTION) + ' ' + time_str + '\n';
	}

	Serial.println("*** GRAPHITE PACKAGE BEGIN ***");
	Serial.print(message);
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
	client.println(message);
	// TODO (k.german): Ensure whether packet reaches its destination point or not.
	// Wait for client.available()?
	client.stop();
}

void initOTA(){
	// Port defaults to 8266
	ArduinoOTA.setPort(OTA_PORT);

	ArduinoOTA.setHostname(MODULE_NAME);
	ArduinoOTA.setPassword(OTA_PASS);

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
