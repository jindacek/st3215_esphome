import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

from . import st3215_ns, St3215Servo

DEPENDENCIES = ["uart"]

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(St3215Servo),
    cv.Required("uart_id"): cv.use_id(cg.UARTDevice),
    cv.Optional("servo_id", default=1): cv.int_range(min=0, max=253),
    cv.Optional("max_angle", default=240.0): cv.float_range(min=1.0, max=3600.0),
    cv.Optional("turns_full_open", default=0.0): cv.float_,
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    uart = await cg.get_variable(config["uart_id"])
    cg.add(var.set_parent(uart))

    cg.add(var.set_servo_id(config["servo_id"]))
    cg.add(var.set_max_angle(config["max_angle"]))
    cg.add(var.set_turns_full_open(config["turns_full_open"]))
