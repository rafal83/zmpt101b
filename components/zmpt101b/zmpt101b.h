#ifndef ZMPT101B_h
#define ZMPT101B_h

#include <Arduino.h>

#define DEFAULT_FREQUENCY 50.0f
#define DEFAULT_SENSITIVITY 500.0f

#if defined(AVR)
	#define ADC_SCALE 1023.0f
	#define VREF 5.0f
#elif defined(ESP8266)
	#define ADC_SCALE 1023.0
	#define VREF 3.3
#elif defined(ESP32)
	#define ADC_SCALE 4095.0
	#define VREF 3.3
#endif

namespace esphome {
namespace zmpt101b {


class ZMPT101B
{
public:
	ZMPT101B (uint8_t pin_, uint16_t frequency_ = DEFAULT_FREQUENCY, float sensitivity_ = DEFAULT_SENSITIVITY);
	float 	 getRmsVoltage(uint8_t loopCount = 1);

private:
  uint8_t  pin;
	uint32_t period;
	float 	 sensitivity;
	int 	 getZeroPoint();
	int		zeroPoint = 0;
};

}
}

#endif
