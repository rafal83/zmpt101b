#include "zmpt101b.h"

namespace esphome {
namespace zmpt101b {

/// @brief ZMPT101B constructor
/// @param pin analog pin that ZMPT101B connected to.
/// @param frequency AC system frequency
ZMPT101B::ZMPT101B(uint8_t pin_, uint16_t frequency_, float sensitivity_)
{
	this->pin = pin_;
	this->period = 1000000 / frequency_;
	
	this->sensitivity = sensitivity_;

}

/// @brief Calculate zero point
/// @return zero / center value
int ZMPT101B::getZeroPoint()
{
	uint32_t Vsum = 0;
	uint32_t measurements_count = 0;
	uint32_t t_start = micros();

	while (micros() - t_start < this->period)
	{
		Vsum += analogRead(pin);
		measurements_count++;
	}

	return Vsum / measurements_count;
}

/// @brief Calculate root mean square (RMS) of AC valtage
/// @param loopCount Loop count to calculate
/// @return root mean square (RMS) of AC valtage
float ZMPT101B::getRmsVoltage(uint8_t loopCount)
{
	double readingVoltage = 0.0f;

	for (uint8_t i = 0; i < loopCount; i++)
	{
		int zeroPoint = this->getZeroPoint();

		int32_t Vnow = 0;
		uint32_t Vsum = 0;
		uint32_t measurements_count = 0;
		uint32_t t_start = micros();
		uint32_t Zsum = 0;

		while (micros() - t_start < this->period)
		{
			uint32_t analogValue = analogRead(pin);
			Zsum += analogValue; // used to re-calculate zeroPoint (every cycle) to (better) handle shifting vcc values
			
			Vnow = analogValue - zeroPoint;
			Vsum += (Vnow * Vnow);
			measurements_count++;
		}
		zeroPoint = Zsum / measurements_count; // new value
		
		readingVoltage += sqrt(Vsum / measurements_count) / ADC_SCALE * VREF * this->sensitivity;
	}

	return readingVoltage / loopCount;
}

}
}
