#ifndef ZMPT101B_h
#define ZMPT101B_h

// Détecte ESP-IDF "pur" (sans Arduino)
#if defined(ESP_PLATFORM) && !defined(ARDUINO)
  #define ZMPT_USE_ESPIDF
#endif

#define DEFAULT_FREQUENCY   50.0f
#define DEFAULT_SENSITIVITY 500.0f   // à ajuster/calibrer selon ton module

// --- Chemins Arduino (inchangés) ---
#if !defined(ZMPT_USE_ESPIDF)
  #include <Arduino.h>
  #if defined(AVR)
    #define ADC_SCALE 1023.0f
    #define VREF      5.0f
  #elif defined(ESP8266)
    #define ADC_SCALE 1023.0f
    #define VREF      3.3f
  #elif defined(ESP32)
    #define ADC_SCALE 4095.0f
    #define VREF      3.3f
  #endif
#endif

// --- Chemins ESP-IDF ---
#ifdef ZMPT_USE_ESPIDF
  #include "esp_timer.h"
  #include "esp_err.h"
  #include "esp_log.h"
  #include "esp_adc/adc_oneshot.h"
  #include "esp_adc/adc_cali.h"
  #include "esp_adc/adc_cali_scheme.h"
  #include "driver/gpio.h"
#endif

namespace esphome {
namespace zmpt101b {

class ZMPT101B {
public:
  // Constructeur existant (conserve la compat Arduino & ton sensor)
  ZMPT101B (uint8_t pin_, uint16_t frequency_ = DEFAULT_FREQUENCY, float sensitivity_ = DEFAULT_SENSITIVITY);

  // Optionnel : constructeur explicite ESP-IDF (si tu veux forcer unit/channel)
#ifdef ZMPT_USE_ESPIDF
  ZMPT101B (adc_unit_t unit, adc_channel_t channel, uint16_t frequency_ = DEFAULT_FREQUENCY, float sensitivity_ = DEFAULT_SENSITIVITY);
#endif

  float getRmsVoltage(uint8_t loopCount = 1);

private:
  // paramètres communs
  uint32_t period;
  float    sensitivity;
  int      getZeroPoint();
  int      zeroPoint = 0;

#if !defined(ZMPT_USE_ESPIDF)
  // Arduino
  uint8_t  pin;
  int      read_raw_();
#else
  // ESP-IDF
  // Remarque: on accepte "pin" pour rester compatible avec ton sensor,
  // puis on tente de le mapper vers (unit, channel) si possible (ESP32 classique).
  uint8_t  pin = 0xFF;

  adc_unit_t    unit_    = ADC_UNIT_1;
  adc_channel_t channel_ = ADC_CHANNEL_0;
  adc_oneshot_unit_handle_t adc_ = nullptr;

  adc_cali_handle_t cali_ = nullptr;
  bool              cali_enabled_ = false;

  void init_adc_from_pin_();
  void init_adc_(adc_unit_t u, adc_channel_t ch);
  int  read_raw_();          // renvoie raw si pas calibré, mV si calibré
  bool map_gpio_to_adc1_(gpio_num_t gpio, adc_channel_t &ch); // ESP32 classique (ADC1)
#endif
};

} // namespace zmpt101b
} // namespace esphome

#endif
