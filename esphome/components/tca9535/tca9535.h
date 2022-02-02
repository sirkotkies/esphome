#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace tca9535 {

class TCA9535Component : public Component, public i2c::I2CDevice {
  

 public:
  TCA9535Component() = default;


  /// Check i2c availability and setup masks
  void setup() override;
  /// Helper function to read the value of a pin.
  bool digital_read(uint8_t pin);
  /// Helper function to write the value of a pin.
  void digital_write(uint8_t pin, bool value);
  /// Helper function to set the pin mode of a pin.
  void pin_mode(uint8_t pin, gpio::Flags flags);

  float get_setup_priority() const override;

  void dump_config() override;

 protected:
  bool read_gpio_();

  bool write_gpio_();

  bool write_gpio_mode_();

  /// Mask for the pin mode - 1 means output, 0 means input
  uint16_t mode_mask_{0x00};
  /// The mask to write as output state - 1 means HIGH, 0 means LOW
  uint16_t output_mask_{0x00};
  /// The state read in read_gpio_ - 1 means HIGH, 0 means LOW
  uint16_t input_mask_{0x00};
   /// Polarity inversion register. BIT '1' inverts input polarity(not used bud provided from harware)
  uint16_t polarity_mask_{0x00};

  enum TCA9535Register{
    TCA9535_INPUT_REG0      =	0x00,		/*!< Input status register */
    TCA9535_INPUT_REG1      =	0x01,		/*!< Input status register */
    TCA9535_OUTPUT_REG0	    =	0x02,		/*!< Output register to change state of output BIT set to 1, output set HIGH */
    TCA9535_OUTPUT_REG1		  =	0x03,		/*!< Output register to change state of output BIT set to 1, output set HIGH */
    TCA9535_POLARITY_REG0   =	0x04,		/*!< Polarity inversion register. BIT '1' inverts input polarity of register 0x00 */
    TCA9535_POLARITY_REG1   =	0x05,		/*!< Polarity inversion register. BIT '1' inverts input polarity of register 0x00 */
    TCA9535_CONFIG_REG0	    =	0x06,		/*!< Configuration register. BIT = '1' sets port to input BIT = '0' sets port to output */
    TCA9535_CONFIG_REG1	    =	0x07		/*!< Configuration register. BIT = '1' sets port to input BIT = '0' sets port to output */
  };
};

/// Helper class to expose a TCA9535 pin as an internal input GPIO pin.
class TCA9535GPIOPin : public GPIOPin {
 public:
  void setup() override;
  void pin_mode(gpio::Flags flags) override;
  bool digital_read() override;
  void digital_write(bool value) override;
  std::string dump_summary() const override;

  void set_parent(TCA9535Component *parent) { parent_ = parent; }
  void set_pin(uint8_t pin) { pin_ = pin; }
  void set_inverted(bool inverted) { inverted_ = inverted; }
  void set_flags(gpio::Flags flags) { flags_ = flags; }

 protected:
  TCA9535Component *parent_;
  uint8_t pin_;
  bool inverted_;
  gpio::Flags flags_;
};

}  // namespace TCA9535
}  // namespace esphome
