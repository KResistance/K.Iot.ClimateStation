#include <Arduino.h>
#include <SoftwareSerial.h>

namespace mhz19b_commands {
	const byte WRITE[9]{0xFF, 0x01, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	const byte MEASURE[9]{0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
	const byte TURN_OFF_SELF_CALIBRATION[8]{0xFF, 0x01, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00};
}

class MHZ19BSensor
{
public:
	struct Data
	{
		int32_t co2_uart;
		int32_t co2_pwm;
	};

private:
	SoftwareSerial sw_serial_;
	uint8_t pwm_pin_;
	Data data_{};

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

		const byte setrange_command[8] = {0xFF, 0x01, 0x99, 0x00, 0x00, 0x00, 0x13, 0x88}; // задаёт диапазон 0 - 5000ppm
		sw_serial_.write(setrange_command, 9);
		sw_serial_.write(calc_crc(setrange_command));
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

	bool measure_by_uart()
	{
		Serial.println("# MH-Z19B::measure_by_uart");
		data_.co2_uart = 0;

		while (sw_serial_.available() > 0)
		{
			sw_serial_.read();
		}

		sw_serial_.write(mhz19b_commands::MEASURE, 9);
		sw_serial_.flush();

		unsigned char measure_response[9]{};
		sw_serial_.readBytes(measure_response, 9);
		byte crc = calc_crc(measure_response);

		if (!(measure_response[0] == 0xFF && measure_response[1] == 0x86 && measure_response[8] == crc))
		{
			Serial.println("CRC error: " + String(crc) + " / "+ String(measure_response[8]));
			return false;
		}

		uint16_t responseHigh{ measure_response[2] };
		uint16_t responseLow{ measure_response[3] };
		data_.co2_uart = (responseHigh << 8) + responseLow;
		data_.co2_uart = (data_.co2_uart / 5) * 2;

		Serial.print("ppm =\t");
		Serial.println(data_.co2_uart);

		return true;
	}

	void measure_by_pwm()
	{
		Serial.println("# MH-Z19B::measure_by_pwm");
		data_.co2_pwm = 0;
		unsigned long th{ 0 };

		while (th == 0)
		{
			th = pulseIn(pwm_pin_, HIGH, 1004000) / 1000;
			unsigned long tl = 1004 - th;
			data_.co2_pwm =  2000 * (th - 2) / (th + tl - 4); // Range between 0 and 2000 ppm.
		}

		Serial.print("ppm =\t");
		Serial.println(data_.co2_pwm);
	}

	const Data& data() const noexcept
	{
		return data_;
	}

private:
	byte calc_crc(const byte* data)
	{
		byte crc = 0;

		for (byte i = 1; i < 8; i++)
		{
			crc += data[i];
		}

		crc = ~crc + 1;

		return crc;
	}
};