import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import i2c
from esphome.const import (
    CONF_ID,
    CONF_INPUT,
    CONF_NUMBER,
    CONF_MODE,
    CONF_INVERTED,
    CONF_OUTPUT,
)

DEPENDENCIES = ["i2c"]
MULTI_CONF = True

twr9535_ns = cg.esphome_ns.namespace("twr9535")

TWR9535Component = twr9535_ns.class_("TWR9535Component", cg.Component, i2c.I2CDevice)
TWR9535GPIOPin = twr9535_ns.class_("TWR9535GPIOPin", cg.GPIOPin)

CONF_TWR9535 = "twr9535"

CONFIG_SCHEMA = (
    cv.Schema({cv.Required(CONF_ID): cv.declare_id(TWR9535Component)})
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x21))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)


def validate_mode(value):
    if not (value[CONF_INPUT] or value[CONF_OUTPUT]):
        raise cv.Invalid("Mode must be either input or output")
    if value[CONF_INPUT] and value[CONF_OUTPUT]:
        raise cv.Invalid("Mode must be either input or output")
    return value


TWR9535_PIN_SCHEMA = cv.All(
    {
        cv.GenerateID(): cv.declare_id(TWR9535GPIOPin),
        cv.Required(CONF_TWR9535): cv.use_id(TWR9535Component),
        cv.Required(CONF_NUMBER): cv.int_range(min=0, max=17),
        cv.Optional(CONF_MODE, default={}): cv.All(
            {
                cv.Optional(CONF_INPUT, default=False): cv.boolean,
                cv.Optional(CONF_OUTPUT, default=False): cv.boolean,
            },
            validate_mode,
        ),
        cv.Optional(CONF_INVERTED, default=False): cv.boolean,
    }
)


@pins.PIN_SCHEMA_REGISTRY.register("twr9535", TWR9535_PIN_SCHEMA)
async def twr9535_pin_to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    parent = await cg.get_variable(config[CONF_TWR9535])

    cg.add(var.set_parent(parent))

    num = config[CONF_NUMBER]
    cg.add(var.set_pin(num))
    cg.add(var.set_inverted(config[CONF_INVERTED]))
    cg.add(var.set_flags(pins.gpio_flags_expr(config[CONF_MODE])))
    return var
