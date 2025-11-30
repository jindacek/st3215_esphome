import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, switch
from esphome.const import CONF_ID, CONF_UART_ID, UNIT_DEGREES, UNIT_PERCENT

DEPENDENCIES = ["uart", "sensor", "switch"]
AUTO_LOAD = ["switch"]

st3215_ns = cg.esphome_ns.namespace("st3215_servo")
St3215Servo = st3215_ns.class_("St3215Servo", cg.Component, uart.UARTDevice)
St3215TorqueSwitch = st3215_ns.class_("St3215TorqueSwitch", switch.Switch, cg.Component)

CONF_SERVO_ID = "servo_id"
CONF_TURNS_FULL_OPEN = "turns_full_open"
CONF_MAX_ANGLE = "max_angle"
CONF_TORQUE_SWITCH = "torque_switch"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(St3215Servo),
            cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
            cv.Required(CONF_SERVO_ID): cv.int_range(min=0, max=100),
            cv.Optional(CONF_MAX_ANGLE, default=240.0): cv.float_,
            cv.Optional(CONF_TURNS_FULL_OPEN, default=0.0): cv.float_,
            cv.Optional("angle"): sensor.sensor_schema(unit_of_measurement=UNIT_DEGREES),
            cv.Optional("turns"): sensor.sensor_schema(),
            cv.Optional("percent"): sensor.sensor_schema(unit_of_measurement=UNIT_PERCENT),
            cv.Optional("calib_state"): sensor.sensor_schema(),
            cv.Optional(CONF_TORQUE_SWITCH): switch.switch_schema(St3215TorqueSwitch),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_servo_id(config[CONF_SERVO_ID]))
    cg.add(var.set_max_angle(config[CONF_MAX_ANGLE]))
    cg.add(var.set_turns_full_open(config[CONF_TURNS_FULL_OPEN]))

    if "angle" in config:
        sens = await sensor.new_sensor(config["angle"])
        cg.add(var.set_angle_sensor(sens))

    if "turns" in config:
        sens = await sensor.new_sensor(config["turns"])
        cg.add(var.set_turns_sensor(sens))

    if "percent" in config:
        sens = await sensor.new_sensor(config["percent"])
        cg.add(var.set_percent_sensor(sens))

    if "calib_state" in config:
        sens = await sensor.new_sensor(config["calib_state"])
        cg.add(var.set_calib_state_sensor(sens))

    if CONF_TORQUE_SWITCH in config:
        sw = await switch.new_switch(config[CONF_TORQUE_SWITCH])
        await cg.register_component(sw, config[CONF_TORQUE_SWITCH])
        cg.add(sw.set_parent(var))
        cg.add(var.set_torque_switch(sw))
        
