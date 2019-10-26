#include <TroykaMQ.h>

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