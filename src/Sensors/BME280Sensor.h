#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

class BME280Sensor
{
public:
	using SensorSampling = Adafruit_BME280::sensor_sampling;
	struct Data
	{
		float temperature{ NAN };
		float humidity{ NAN };
		float pressure{ NAN };
	};

private:
	SensorSampling sampling_;
	uint8_t address_;
	Adafruit_BME280 bme_{}; // SCL=D1, SDA=D2, VIN=3.3V, GND=GND
	Data data_{};

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
		Serial.printf("Temperature =\t%e\n", data_.temperature);
		Serial.printf("Humidity =\t%e\n", data_.humidity);
		Serial.printf("Pressure =\t%e\n", data_.pressure);
	}

	void measure()
	{
		data_ = {};
		bme_.takeForcedMeasurement();
		data_.temperature = bme_.readTemperature();
		data_.humidity = bme_.readHumidity();
		data_.pressure = bme_.readPressure() / 100.0F;
	}

	const Data& data() const noexcept
	{
		return data_;
	}
};