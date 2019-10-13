#include <Arduino.h>
#include <ArduinoOTA.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SoftwareSerial.h>
#include <DHTesp.h>
#include <TroykaMQ.h>
#include "sntp.h"

// !IMPORTANT! TODO (k.german): Send data to graphite from MH-Z19B and BME280.

// TODO (k.german): Add debug DEFINEs, IFDEFs, etc.

// TODO (k.german): Enable/disable WiFi and sending data by #define?

// TODO (k.german): Overused delay function. Schedule measuring and remember
// to yield in order to get WiFi's and other peripheral's stuff done.

// Parameters
#define MODULE_NAME "esp8266-climate-station"

#define BME280_SCL_PIN D1 // also GPIO5/SCL
#define BME280_SDA_PIN D2 // also GPIO4/SDA

// NOTE (k.german): SDD3 used in QIO flashing mode. Therefore, use DIO mode
// instead to free this pin. It can make troubles on some NodeMCU's devkits.
#define DHT_OUT_PIN 10 // SDD3/GPIO10
#define DHT_SAMPLE_TIMES 5
#define DHT_SAMPLE_INTERVAL 20 // ms

#define MQ135_PIN A0 // also ADC0
#define MQ135_PIN_CALIBRATION 45

#define Z19B_PWM_PIN D5 // also GPIO14/SCLK
#define Z19B_RX_PIN D6  // also GPIO12/MISO
#define Z19B_TX_PIN D7  // also GPIO13/MOSI/RXD2

WiFiClient client{};
time_t sampling_time{};

class BME280Sensor
{
public:
	using SensorSampling = Adafruit_BME280::sensor_sampling;

private:
	SensorSampling sampling_;
	uint8_t address_;
	Adafruit_BME280 bme_{}; // SCL=D1, SDA=D2, VIN=3.3V, GND=GND
	float temperature_{ NAN };
	float humidity_{ NAN };
	float pressure_{ NAN };

public:
	BME280Sensor(
		SensorSampling sampling = SensorSampling::SAMPLING_X1,
		uint8_t address = BME280_ADDRESS_ALTERNATE // or BME280_ADDRESS
	) : sampling_{ sampling }, address_{ address }
	{}

	void setup()
	{
		Serial.println("# BME280::setup");

		if (!bme_.begin(address_))
		{
			Serial.println(
				"Could not find a valid BME280 sensor."
				" Please, check wiring, address or sensor ID."
			);
			Serial.printf("- Address: 0x%x\n", address_ & 0xFF);
			Serial.printf("- Sensor ID: 0x%x\n", bme_.sensorID());
			Serial.println(
				"  ID 0xFF       probably means a bad address, a BMP 180 or BMP 085.\n"
				"  ID 0x56-0x58  represents a BMP 280.\n"
				"  ID 0x60       represents a BME 280.\n"
				"  ID 0x61       represents a BME 680."
			);

			while (1); // QUESTION (k.german): Why is it here?
		}

		bme_.setSampling(
			Adafruit_BME280::MODE_FORCED,
			sampling_,
			sampling_,
			sampling_,
			Adafruit_BME280::FILTER_OFF
		);
	}

	void print_measurements()
	{
		Serial.println("# BME280::info");
		Serial.printf("Temperature =\t%e\n", temperature_);
		Serial.printf("Humidity =\t%e\n", humidity_);
		Serial.printf("Pressure =\t%e\n", pressure_);
	}

	void measure()
	{
		bme_.takeForcedMeasurement();
		temperature_ = bme_.readTemperature();
		humidity_ = bme_.readHumidity();
		pressure_ = bme_.readPressure() / 100.0F;
	}

	float temperature()
	{
		return temperature_;
	}

	float humidity()
	{
		return humidity_;
	}

	float pressure()
	{
		return pressure_;
	}
};

// NOTE (k.german): Used for checking response MH-Z19B.
byte calc_crc(const byte* data)
{
	byte i;
	byte crc = 0;

	for (i = 1; i < 8; i++)
	{
		crc += data[i];
	}

	crc = 255 - crc;
	crc++;

	return crc;
}

namespace mhz19b_commands {
	const byte WRITE[9]{0xFF, 0x01, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	const byte MEASURE[9]{0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
	const byte TURN_OFF_SELF_CALIBRATION[8]{0xFF, 0x01, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00};
}

class MHZ19BSensor
{
private:
	SoftwareSerial sw_serial_;
	uint8_t pwm_pin_;

public:
	MHZ19BSensor(int rx_pin, int tx_pin, uint8_t pwm_pin)
		: sw_serial_{ rx_pin, tx_pin }, pwm_pin_{ pwm_pin }
		{}

	void setup()
	{
		sw_serial_.begin(9600);
		pinMode(pwm_pin_, INPUT);

		/*
		Source: https://revspace.nl/MHZ19 и https://github.com/strange-v/MHZ19/blob/ba883d166eaba26b0d6f68c1cd0e664cd405a1ad/MHZ19.cpp#L58
		  1000 ppm range: 0xFF, 0x01, 0x99, 0x00, 0x00, 0x00, 0x03, 0xE8,
		  2000 ppm range: 0xFF, 0x01, 0x99, 0x00, 0x00, 0x00, 0x07, 0xD0, 0x8F
		  3000 ppm range: 0xFF, 0x01, 0x99, 0x00, 0x00, 0x00, 0x0B, 0xB8,
		  5000 ppm range: 0xFF, 0x01, 0x99, 0x00, 0x00, 0x00, 0x13, 0x88, 0xCB
		  10000 ppm range: 0xFF, 0x01, 0x99, 0x00, 0x00, 0x00, 0x27, 0x10,
		*/

		// Этот вариант ("A") с записью команды в 6й и 7й байт - работает
		//           bytes:                          3     4           6     7
		const byte setrangeA_cmd[8] = {0xFF, 0x01, 0x99, 0x00, 0x00, 0x00, 0x13, 0x88}; // задаёт диапазон 0 - 5000ppm
		sw_serial_.write(setrangeA_cmd,9);
		sw_serial_.write(calc_crc(setrangeA_cmd));
		sw_serial_.flush();

		byte setrangeA_response[9];
		sw_serial_.readBytes(setrangeA_response, 9);
		byte setrangeA_crc = calc_crc(setrangeA_response);

		if (!(setrangeA_response[0] == 0xFF && setrangeA_response[1] == 0x99 && setrangeA_response[8] == setrangeA_crc))
		{
			Serial.println("Range CRC error: " + String(setrangeA_crc) + " / "+ String(setrangeA_response[8]) + " (bytes 6 and 7)");
		}
		else
		{
			Serial.println("Range was set! (bytes 6 and 7)");
		}

		delay(1000);
		turn_off_self_calibration();
		delay(1000);
	}

	void turn_off_self_calibration()
	{
		sw_serial_.write(mhz19b_commands::TURN_OFF_SELF_CALIBRATION, 8);
		sw_serial_.write(calc_crc(mhz19b_commands::TURN_OFF_SELF_CALIBRATION));
		sw_serial_.flush();

		unsigned char response[9];
		sw_serial_.readBytes(response, 9);

		byte response_crc = calc_crc(response);

		if (!(response[0] == 0xFF && response[1] == 0x79 && response[8] == response_crc))
		{
			Serial.println("Range CRC error: " + String(response_crc) + " / "+ String(response[8]));
		}
		else
		{
			Serial.println("Self calibration was turned off!");
		}
	}

	void measure() {
		unsigned char measure_response[9]{};

		Serial.println("# MH-Z19B::measure");

		// Check CO2 concentration by UART
		while (sw_serial_.available() > 0)
		{
			sw_serial_.read();
		}

		sw_serial_.write(mhz19b_commands::MEASURE, 9);
		sw_serial_.flush();
		sw_serial_.readBytes(measure_response, 9);
		byte crc = calc_crc(measure_response);

		if (!(measure_response[0] == 0xFF && measure_response[1] == 0x86 && measure_response[8] == crc))
		{
			Serial.println("CRC error: " + String(crc) + " / "+ String(measure_response[8]));
		}

		// FIXME: static_cast or reinterpret_cast?
		unsigned int responseHigh{ static_cast<unsigned int>(measure_response[2]) };
		unsigned int responseLow{ static_cast<unsigned int>(measure_response[3]) };
		unsigned long ppm = 256 * responseHigh + responseLow;

		// Check CO2 concentration by PWM
		unsigned long th{ 0 }, ppm2{ 0 };

		while (th == 0)
		{
			th = pulseIn(pwm_pin_, HIGH, 1004000) / 1000;
			unsigned long tl = 1004 - th;
			ppm2 =  2000 * (th-2)/(th+tl-4); // расчёт для диапазона от 0 до 2000ppm
		}

		Serial.print("ppm (UART) =\t");
		Serial.println(ppm);
		Serial.print("ppm (two fifths of it; real value) =\t");
		Serial.println((ppm / 5) * 2);
		Serial.print("Time while PWM was HIGH (ms) =\t");
		Serial.println(th);
		Serial.print("ppm (PWM; valid range) =\t");
		Serial.println(ppm2);
	}
};

class DHT22Sensor
{
public:
	using Data = TempAndHumidity;

private:
	DHTesp dht_{};
	uint8_t out_pin_;

	int sampling_times_;
	int sampling_interval_;

	Data data_{};
	int humidity_samples_{};
	int temperature_samples_{};

public:
	DHT22Sensor(uint8_t out_pin, int sampling_times = 1, int sampling_interval = 20)
		: out_pin_{ out_pin }
		, sampling_times_{ sampling_times }
		, sampling_interval_{ sampling_interval }
	{}

	void setup()
	{
		dht_.setup(out_pin_, DHTesp::DHT22);
	}

	void measure()
	{
		Serial.println("# DHT22::measure");

		data_ = {};
		humidity_samples_ = 0;
		temperature_samples_ = 0;

		for (int i = 0; i < sampling_times_; ++i)
		{
			const auto& dht_sample = dht_.getTempAndHumidity();

			if (!isnan(dht_sample.temperature))
			{
				Serial.printf("Temperature sample = %e\n", dht_sample.temperature);
				data_.temperature += dht_sample.temperature;
				++temperature_samples_;
			}
			else
			{
				Serial.println("Temperature sample = NAN");
			}

			if (!isnan(dht_sample.humidity))
			{
				Serial.printf("Humidity sample = %e\n", dht_sample.humidity);
				data_.humidity += dht_sample.humidity;
				++humidity_samples_;
			}
			else
			{
				Serial.println("Humidity sample = NAN");
			}

			delay(DHT_SAMPLE_INTERVAL);
		}

		if (humidity_samples_ >= 1)
		{
			data_.humidity /= humidity_samples_;
		}
		else
		{
			data_.humidity = NAN;
		}

		if (temperature_samples_ >= 1)
		{
			data_.temperature /= temperature_samples_;
		}
		else
		{
			data_.temperature = NAN;
		}

		Serial.printf("Humidity = %e\n", data_.humidity);
		Serial.printf("Temperature = %e\n", data_.temperature);
	}

	const Data& data() const noexcept
	{
		return data_;
	}
};

class MQ135Sensor
{
public:
	struct Data
	{
		float co2;
	};

private:
	MQ135 mq135_;
	float calibration_ro_;
	Data data_;

public:
	MQ135Sensor(const uint8_t aout_pin, const float calibration_ro = NAN)
		: mq135_{ aout_pin }, calibration_ro_{ calibration_ro } {}

	void setup()
	{
		Serial.println("# MQ135::setup");
		Serial.printf("Calibration Ro = %e\n", calibration_ro_);

		if (!isnan(calibration_ro_))
		{
			mq135_.calibrate(calibration_ro_);
		}

		Serial.printf("Ro = %e\n", mq135_.getRo());
	};

  void measure(const float temperature = NAN, const float humidity = NAN)
	{
		Serial.println("# MQ135::measure");
		auto& co2 = data_.co2;
		co2 = NAN;

		if (!isnan(temperature) && !isnan(humidity))
		{
			// TODO (k.german): unsigned long? float?
			co2 = mq135_.readCorrectedCO2(temperature, humidity);

			if (!isnan(co2))
			{
				Serial.println("CO2 has been corrected.");
			}
		}

		if (isnan(co2))
		{
			// TODO (k.german): unsigned long? float?
			co2 = mq135_.readCO2();
		}

		Serial.printf("CO2 = %e\n", co2);
	}

	const Data& data() const noexcept
	{
		return data_;
	}
};

BME280Sensor bme280_sensor{};
MHZ19BSensor mhz19b_sensor{ Z19B_RX_PIN, Z19B_TX_PIN, Z19B_PWM_PIN };
DHT22Sensor dht22_sensor{ DHT_OUT_PIN, DHT_SAMPLE_TIMES, DHT_SAMPLE_INTERVAL };
MQ135Sensor mq135_sensor{ MQ135_PIN };

void initOTA();
void collectData();
void sendData();

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

void setup()
{
	Serial.begin(115200);
	Serial.printf("\nStart setup \"%s\"\n", MODULE_NAME);

	mhz19b_sensor.setup();
	bme280_sensor.setup();
	dht22_sensor.setup();
	mq135_sensor.setup();

	// initWiFi();
	initOTA();

	// configTime(0, 0, "time.google.com", "ru.pool.ntp.org");
	// int last = 0;
  //
	// while(sntp_get_current_timestamp() < 1500000000)
	// {
	// 	if((millis() - last) > 1000)
	// 	{
	// 		last = millis();
	// 		Serial.println(sntp_get_current_timestamp());
	// 		Serial.println("Waiting for sntp.");
	// 	}

	// 	yield();
	// }
}

void loop()
{
	ArduinoOTA.handle();
	yield();
	collectData();
	yield();
	// sendData();
	delay(1000);
}

void collectData()
{
	sampling_time = sntp_get_current_timestamp();

	dht22_sensor.measure();
	const auto& dht22_data = dht22_sensor.data();
	mq135_sensor.measure(dht22_data.temperature, dht22_data.humidity);

	// FIXME: No output :C
	mhz19b_sensor.measure();
	bme280_sensor.measure();

	delay(1000);
}

void sendData()
{
	// TODO (k.german): printf to buffer?

	String time_str{ sampling_time };
	String data{};
	const auto& dht22_data = dht22_sensor.data();

	if (!isnan(dht22_data.temperature))
	{
		data += GRAPHITE_PREFIX "t.avg " + String(dht22_data.temperature) + ' ' + time_str + '\n';
	}

	if (!isnan(dht22_data.humidity))
	{
		data += GRAPHITE_PREFIX "h.avg " + String(dht22_data.humidity) + ' ' + time_str + '\n';
	}

	const auto co2 = mq135_sensor.data().co2;

	if (!isnan(co2))
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
