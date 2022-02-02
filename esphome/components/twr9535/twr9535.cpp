#include "twr9535.h"
#include "esphome/core/log.h"

namespace esphome {
namespace twr9535 {

static const char *const TAG = "twr9535";

void TWR9535Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up TWR9535...");
  if (!this->read_gpio_()) {
    ESP_LOGE(TAG, "TWR9535 not available under 0x%02X", this->address_);
    this->mark_failed();
    return;
  }

  this->write_gpio_();
  this->read_gpio_();
}
void TWR9535Component::dump_config() {
  ESP_LOGCONFIG(TAG, "TWR9535:");
  LOG_I2C_DEVICE(this)
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with TWR9535 failed!");
  }
}
bool TWR9535Component::digital_read(uint8_t pin) {
  this->read_gpio_();
  return this->input_mask_ & (1 << pin);
}
void TWR9535Component::digital_write(uint8_t pin, bool value) {
  if (value) {
    this->output_mask_ |= (1 << pin);
  } else {
    this->output_mask_ &= ~(1 << pin);
  }

  this->write_gpio_();
}
void TWR9535Component::pin_mode(uint8_t pin, gpio::Flags flags) {
  if (flags == gpio::FLAG_INPUT) {
    // Clear mode mask bit
    this->mode_mask_ &= ~(1 << pin);
    // Write GPIO to enable input mode
  } else if (flags == gpio::FLAG_OUTPUT) {
    // Set mode mask bit
    this->mode_mask_ |= 1 << pin;
  }
  this->write_gpio_mode_();
}
bool TWR9535Component::write_gpio_mode_() {
  if (this->is_failed())
    return false;
  bool  success;
	success=write_byte_16(TWR9535_CONFIG_REG0,this->mode_mask_);
	return success;
}
bool TWR9535Component::read_gpio_() {
  if (this->is_failed()){
    return false;
  }
  bool success;
	uint8_t data[2];
	data[0]=TWR9535_INPUT_REG0;
	success=write(data, (uint8_t)1) == i2c::ERROR_OK;

	if(success){
		success=read(data, (uint8_t)2) == i2c::ERROR_OK;

		if(success){
      this->input_mask_ = (uint16_t(data[1]) << 8) | (uint16_t(data[0]) << 0);
		}
	}
  if (!success) {
    this->status_set_warning();
    return false;
  }
  this->status_clear_warning();
  return true;
}
bool TWR9535Component::write_gpio_() {
   if (this->is_failed())
    return false;
  bool  success;

	success=write_byte_16(TWR9535_OUTPUT_REG0,output_mask_);
	return success;
  
}
float TWR9535Component::get_setup_priority() const { return setup_priority::IO; }

void TWR9535GPIOPin::setup() { pin_mode(flags_); }
void TWR9535GPIOPin::pin_mode(gpio::Flags flags) { this->parent_->pin_mode(this->pin_, flags); }
bool TWR9535GPIOPin::digital_read() { return this->parent_->digital_read(this->pin_) != this->inverted_; }
void TWR9535GPIOPin::digital_write(bool value) { this->parent_->digital_write(this->pin_, value != this->inverted_); }
std::string TWR9535GPIOPin::dump_summary() const {
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%u via TWR9535", pin_);
  return buffer;
}

}  // namespace twr9535
}  // namespace esphome
