#include <DHTesp.h>

class DHT22Sensor
{
public:
	using Data = TempAndHumidity;

private:
	DHTesp dht_{};
	uint8_t out_pin_;

	int sampling_times_;
	unsigned int sampling_interval_;

	Data data_{};
	int humidity_samples_{};
	int temperature_samples_{};

public:
	DHT22Sensor(
		const uint8_t out_pin, const int sampling_times = 1, const unsigned int sampling_interval = 20)
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

			int waitAfterCollect = millis();
  			while((millis() - waitAfterCollect) < sampling_interval_){
				yield();
			}
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