#include "gree.h"
#include "esphome/core/log.h"

namespace esphome {
namespace gree {

static const char *TAG = "gree.climate";

const uint8_t GREE_AUTO = 0;
const uint8_t GREE_COOL = 1;
const uint8_t GREE_DRY = 2;
const uint8_t GREE_FAN = 3;
const uint8_t GREE_HEAT = 4;

//byte 0
const uint8_t GREE_MODE_MASK = 0b00000111;
const uint8_t GREE_POWER1_MASK = 0b00001000;
const uint8_t GREE_FAN_MASK = 0b00110000;
const uint8_t GREE_FAN_AUTO = 0;
const uint8_t GREE_FAN_MIN = 1;
const uint8_t GREE_FAN_MED = 2;
const uint8_t GREE_FAN_MAX = 3;
const uint8_t GREE_SWING_AUTO_MASK = 0b01000000;
const uint8_t GREE_SLEEP_MASK = 0b10000000;
//byte 1
const uint8_t GREE_TEMP_MASK = 0b00001111;
const uint8_t GREE_TEMP_MIN = 16; // Celsius
const uint8_t GREE_TEMP_MAX = 30; // Celsius
const uint8_t GREE_TIMER_ENABLED_BIT = 0b10000000;
const uint8_t GREE_TIMER_HALF_HR_BIT = 0b00010000;
const uint8_t GREE_TIMER_TENS_HR_MASK = 0b01100000;
const uint8_t GREE_TIMER1_MASK = kGreeTimerTensHrMask | kGreeTimerHalfHrBit;
const uint16_t GREE_TIMER_MAX = 24 * 60;

// Byte 2
const uint8_t GREE_TIMER_HOURS_MASK = 0b00001111;
const uint8_t GREE_TURBO_MASK = 0b00010000;
const uint8_t GREE_LIGHT_MASK = 0b00100000;
// This might not be used
const uint8_t GREE_POWER2_MASK = 0b01000000;
const uint8_t GREE_X_FAN_MASK = 0b10000000;
// Byte 4
const uint8_t GREE_SWING_POS_MASK = 0b00001111;
// Byte 5
const uint8_t GREE_I_FEEL_MASK = 0b00000100;
const uint8_t GREE_WIFI_MASK = 0b01000000;

const uint8_t GREE_SWING_LAST_POS = 0b00000000;
const uint8_t GREE_SWING_AUTO = 0b00000001;
const uint8_t GREE_SWING_UP = 0b00000010;
const uint8_t GREE_SWING_MIDDLE_UP = 0b00000011;
const uint8_t GREE_SWING_MIDDLE = 0b00000100;
const uint8_t GREE_SWING_MIDDLE_DOWN = 0b00000101;
const uint8_t GREE_SWING_DOWN = 0b00000110;
const uint8_t GREE_SWING_DOWN_AUTO = 0b00000111;
const uint8_t GREE_SWING_MIDDLE_AUTO = 0b00001001;
const uint8_t GREE_SWING_UP_AUTO = 0b00001011;

// Temperature
const uint8_t GREE_TEMP_RANGE = GREE_TEMP_MAX - GREE_TEMP_MIN + 1;

// Constants
static const uint16_t GREE_HEADER_MARK = 9000;
static const uint16_t GREE_HEADER_SPACE = 4500;
static const uint16_t GREE_BIT_MARK = 620;
static const uint16_t GREE_ONE_SPACE = 1600;
static const uint16_t GREE_ZERO_SPACE = 540;
static const uint16_t GREE_MSG_SPACE = 19000;
static const uint8_t GREE_BLOCK_FOOTER = 0b010;
static const uint8_t GREE_BLOCK_FOOTER_BITS = 3;

const uint16_t GREE_STATE_LENGTH = 8
const uint16_t GREE_BITS = GREE_STATE_LENGTH * 8;

climate::ClimateTraits GreeClimate::traits() {
  auto traits = climate::ClimateTraits();
  traits.set_supports_current_temperature(this->sensor_ != nullptr);
  traits.set_supports_auto_mode(true);
  traits.set_supports_cool_mode(this->supports_cool_);
  traits.set_supports_heat_mode(this->supports_heat_);
  traits.set_supports_two_point_target_temperature(false);
  traits.set_supports_away(false);
  traits.set_visual_min_temperature(16);
  traits.set_visual_max_temperature(30);
  traits.set_visual_temperature_step(1);
  return traits;
}

void GreeClimate::setup() {
  if (this->sensor_) {
    this->sensor_->add_on_state_callback([this](float state) {
      this->current_temperature = state;
      // current temperature changed, publish state
      this->publish_state();
    });
    this->current_temperature = this->sensor_->state;
  } else
    this->current_temperature = NAN;
  // restore set points
  auto restore = this->restore_state_();
  if (restore.has_value()) {
    restore->apply(this);
  } else {
    // restore from defaults
    this->mode = climate::CLIMATE_MODE_OFF;
    // initialize target temperature to some value so that it's not NAN
    this->target_temperature = (uint8_t) roundf(clamp(this->current_temperature, GREE_TEMP_MIN, GREE_TEMP_MAX));
  }
  // never send nan as temperature. HA will disable the user to change the temperature.
  if (isnan(this->target_temperature))
    this->target_temperature = 24;
}

void GreeClimate::control(const climate::ClimateCall &call) {
  if (call.get_mode().has_value())
    this->mode = *call.get_mode();
  if (call.get_target_temperature().has_value())
    this->target_temperature = *call.get_target_temperature();

  this->transmit_state_();
  this->publish_state();
}

void GreeClimate::transmit_state_() {
	uint8_t remote_state[GREE_STATE_LENGTH] = {0};

	// A known good state Power On, Fan Auto, Mode Auto, 25C.
	remote_state[0] |= GREE_POWER1_MASK;
	remote_state[1] = 0x09;
	remote_state[2] |= GREE_POWER2_MASK;
	remote_state[3] = 0x50;
	remote_state[4] = 0x0;
	remote_state[5] = 0x20;
	remote_state[6] = 0x0;
	remote_state[7] = 0x50;

	swith (this->mode) {
		case climate::CLIMATE_MODE_AUTO:
			remote_state[0] &= ~GREE_MODE_MASK;
			remote_state[0] |= GREE_AUTO;
			break;
		case climate::CLIMATE_MODE_COOL:
			remote_state[0] &= ~GREE_MODE_MASK;
			remote_state[0] |= GREE_COOL;
			break;
		case climate::CLIMATE_MODE_HEAT:
			remote_state[0] &= ~GREE_MODE_MASK;
			remote_state[0] |= GREE_HEAT;
			break;
		case climate::CLIMATE_MODE_OFF:
		default:
			remote_state[0] &= ~GREE_POWER1_MASK;
			remote_state[2] &= ~GREE_POWER2_MASK;
			break;
			// TODO: CLIMATE_MODE_FAN_ONLY, CLIMATE_MODE_DRY not implemented in esphome
	}

	// TODO: missing support for fan speed
	
	// Set temperature
	uint8_t safecelsius = std::max((uint8_t) this->target_temperature, GREE_TEMP_MIN);
	safecelsius = std::min(safecelsius, GREE_TEMP_MAX);
    remote_state[1] = (remote_state[1] & ~GREE_TEMP_MASK) |
                      (safecelsius - GREE_TEMP_MIN);

    ESP_LOGV(TAG, "Sending gree code: %u", remote_state);

    auto transmit = this->transmitter_->transmit();
    auto data = transmit.get_data();

    data->set_carrier_frequency(38000);
  
    uint16_t repeat = 1;

    for (uint16_t r = 0; r <= repeat; r++) {
      // Header
      data->mark(GREE_HEADER_MARK);
      data->space(GREE_HEADER_SPACE);
      // Data
	  for (int16_t i = 8; i <= GREE_BITS; i += 8) {
	    data->mark(GREE_BIT_MARK);
	    bool bit = i & (1 << j);
	    data->space(bit ? GREE_ONE_SPACE : GREE_ZERO_SPACE);
        if (i == GREE_BITS / 2) {
          // Send the mid-message Footer.
		  data->mark(GREE_BIT_MARK);
		  data->space(GREE_ZERO_SPACE);
		  data->mark(GREE_BIT_MARK);
		  data->space(GREE_ONE_SPACE);
		  data->mark(GREE_BIT_MARK);
		  data->space(GREE_ZERO_SPACE);
		
		  // Message space
		  data->mark(GREE_BIT_MARK);
		  data->space(GREE_MSG_SPACE);
        }
      }
      // Footer
      data->mark(GREE_BIT_MARK);
      data->space(GREE_MSG_SPACE);  // Pause before repeating
    }

    transmit.perform();
}

}  // namespace gree
}  // namespace esphome
