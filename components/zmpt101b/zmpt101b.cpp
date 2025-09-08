#include "zmpt101b.h"
#include <math.h>

namespace esphome {
namespace zmpt101b {

// ----------------- Constructeurs -----------------
ZMPT101B::ZMPT101B(uint8_t pin_, uint16_t frequency_, float sensitivity_)
  : period((uint32_t)(1000000UL / frequency_)),
    sensitivity(sensitivity_)
{
#if !defined(ZMPT_USE_ESPIDF)
  this->pin = pin_;
#else
  this->pin = pin_;
  init_adc_from_pin_(); // tente un mapping GPIO -> ADC(chan)
#endif
}

#ifdef ZMPT_USE_ESPIDF
ZMPT101B::ZMPT101B(adc_unit_t unit, adc_channel_t channel, uint16_t frequency_, float sensitivity_)
  : period((uint32_t)(1000000UL / frequency_)),
    sensitivity(sensitivity_)
{
  init_adc_(unit, channel);
}
#endif

// ----------------- Lecture brute -----------------
#if !defined(ZMPT_USE_ESPIDF)
int ZMPT101B::read_raw_() {
  return analogRead(this->pin);
}
#else
void ZMPT101B::init_adc_from_pin_() {
  // Valeur par défaut si on ne peut pas mapper proprement
  adc_unit_t u = ADC_UNIT_1;
  adc_channel_t ch = ADC_CHANNEL_0;

  // Tentative de mapping pour ESP32 "classique" (ADC1 uniquement)
  adc_channel_t guessed;
  if (map_gpio_to_adc1_((gpio_num_t)this->pin, guessed)) {
    u = ADC_UNIT_1;
    ch = guessed;
  }
  init_adc_(u, ch);
}

void ZMPT101B::init_adc_(adc_unit_t u, adc_channel_t ch) {
  unit_ = u;
  channel_ = ch;

  adc_oneshot_unit_init_cfg_t unit_cfg = {};
  unit_cfg.unit_id = unit_;
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_));

  adc_oneshot_chan_cfg_t chan_cfg = {};
  chan_cfg.bitwidth = ADC_BITWIDTH_DEFAULT; // 12 bits
  chan_cfg.atten    = ADC_ATTEN_DB_11;      // pleine échelle ~3.6V
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_, channel_, &chan_cfg));

  // Calibrage si dispo
  adc_cali_line_fitting_config_t cali_cfg = {};
  cali_cfg.unit_id = unit_;
  cali_cfg.atten   = chan_cfg.atten;
#if SOC_ADC_SUPPORTED
  cali_cfg.bitwidth = chan_cfg.bitwidth;
#endif
  if (adc_cali_create_scheme_line_fitting(&cali_cfg, &cali_) == ESP_OK) {
    cali_enabled_ = true;
  } else {
    cali_enabled_ = false;
  }
}

int ZMPT101B::read_raw_() {
  int raw = 0;
  ESP_ERROR_CHECK(adc_oneshot_read(adc_, channel_, &raw));

  if (cali_enabled_) {
    int mv = 0;
    if (adc_cali_raw_to_voltage(cali_, raw, &mv) == ESP_OK) {
      return mv; // retourne des millivolts si calibré
    }
  }
  return raw; // sinon, valeur brute (0..4095)
}

// ADC1: GPIO36(0), 37(1), 38(2), 39(3), 32(4), 33(5), 34(6), 35(7)
bool ZMPT101B::map_gpio_to_adc1_(gpio_num_t gpio, adc_channel_t &ch) {
  switch (gpio) {
    case GPIO_NUM_36: ch = ADC_CHANNEL_0; return true;
    case GPIO_NUM_37: ch = ADC_CHANNEL_1; return true;
    case GPIO_NUM_38: ch = ADC_CHANNEL_2; return true;
    case GPIO_NUM_39: ch = ADC_CHANNEL_3; return true;
    case GPIO_NUM_32: ch = ADC_CHANNEL_4; return true;
    case GPIO_NUM_33: ch = ADC_CHANNEL_5; return true;
    case GPIO_NUM_34: ch = ADC_CHANNEL_6; return true;
    case GPIO_NUM_35: ch = ADC_CHANNEL_7; return true;
    default: return false;
  }
}
#endif

// ----------------- Zero point -----------------
int ZMPT101B::getZeroPoint()
{
  uint32_t Vsum = 0;
  uint32_t measurements_count = 0;

#if !defined(ZMPT_USE_ESPIDF)
  uint32_t t_start = micros();
  while (micros() - t_start < this->period)
#else
  uint64_t t_start = esp_timer_get_time();
  while ((uint64_t)esp_timer_get_time() - t_start < this->period)
#endif
  {
    Vsum += read_raw_();
    measurements_count++;
  }

  return measurements_count ? (int)(Vsum / measurements_count) : 0;
}

// ----------------- RMS -----------------
float ZMPT101B::getRmsVoltage(uint8_t loopCount)
{
  double readingVoltage = 0.0;

  for (uint8_t i = 0; i < loopCount; i++)
  {
    int zeroPoint_local = this->zeroPoint;
    if (zeroPoint_local == 0) zeroPoint_local = this->getZeroPoint();

    int32_t Vnow = 0;
    uint32_t Vsum = 0;
    uint32_t measurements_count = 0;
    uint32_t Zsum = 0;

#if !defined(ZMPT_USE_ESPIDF)
    uint32_t t_start = micros();
    while (micros() - t_start < this->period)
#else
    uint64_t t_start = esp_timer_get_time();
    while ((uint64_t)esp_timer_get_time() - t_start < this->period)
#endif
    {
      int analogValue = read_raw_();
      Zsum += analogValue;

      Vnow = analogValue - zeroPoint_local;
      Vsum += (uint32_t)(Vnow * Vnow);
      measurements_count++;
    }

    this->zeroPoint = measurements_count ? (int)(Zsum / measurements_count) : zeroPoint_local;

    // Conversion vers Volts selon la plateforme
#if !defined(ZMPT_USE_ESPIDF)
    // Arduino: raw -> VREF * raw/ADC_SCALE (logique d’origine)
    double v_out_rms = sqrt((double)Vsum / (double)measurements_count) / ADC_SCALE * VREF;
#else
    // ESP-IDF:
    // - si calibré: read_raw_() renvoie des mV -> RMS en mV, puis /1000.
    // - sinon: read_raw_() renvoie raw 0..4095 -> approx V = raw/4095*3.3 (atten 11dB ~ 3.3–3.6V)
    double rms_base = sqrt((double)Vsum / (double)measurements_count);
    double v_out_rms = 0.0;

    if (cali_enabled_) {
      v_out_rms = (rms_base / 1000.0); // mV -> V
    } else {
      v_out_rms = (rms_base / 4095.0) * 3.3;
    }
#endif

    // Applique la sensibilité (V_out -> V_in secteur estimée)
    // Hypothèse: sensitivity en mV/V (module), donc V_in = V_out * (1000 / sensitivity)
    double mains_v_rms = v_out_rms * (1000.0 / (double)this->sensitivity);
    readingVoltage += mains_v_rms;
  }

  return (float)(readingVoltage / loopCount);
}

} // namespace zmpt101b
} // namespace esphome
