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

# 👉 NOVÉ:
CONF_INVERT_DIRECTION = "invert_direction"

_SERVO_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(St3215Servo),
        cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
        cv.Required(CONF_SERVO_ID): cv.int_range(min=0, max=100),
        cv.Optional(CONF_MAX_ANGLE, default=240.0): cv.float_,
        cv.Optional(CONF_TURNS_FULL_OPEN, default=0.0): cv.float_,

        # 👉 NOVÉ: invertace směru
        cv.Optional(CONF_INVERT_DIRECTION, default=False): cv.boolean,
        cv.Optional("ramp_factor", default=1.0): cv.float_range(min=0.5, max=3.5),
        cv.Optional("angle"): sensor.sensor_schema(unit_of_measurement=UNIT_DEGREES),
        cv.Optional("turns"): sensor.sensor_schema(),
        cv.Optional("percent"): sensor.sensor_schema(unit_of_measurement=UNIT_PERCENT),
        cv.Optional("calib_state"): sensor.sensor_schema(),
        cv.Optional(CONF_TORQUE_SWITCH): switch.switch_schema(St3215TorqueSwitch),
    }
).extend(uart.UART_DEVICE_SCHEMA)

CONFIG_SCHEMA = cv.All(cv.ensure_list(_SERVO_SCHEMA))


async def to_code(config):
    for conf in config:
        var = cg.new_Pvariable(conf[CONF_ID])
        await cg.register_component(var, conf)
        await uart.register_uart_device(var, conf)

        cg.add(var.set_servo_id(conf[CONF_SERVO_ID]))
        cg.add(var.set_max_angle(conf[CONF_MAX_ANGLE]))
        cg.add(var.set_turns_full_open(conf[CONF_TURNS_FULL_OPEN]))

        # 👉 NOVÉ:
        cg.add(var.set_invert_direction(conf[CONF_INVERT_DIRECTION]))
        if "ramp_factor" in conf:
            cg.add(var.set_ramp_factor(conf["ramp_factor"]))    
        
        if "angle" in conf:
            sens = await sensor.new_sensor(conf["angle"])
            cg.add(var.set_angle_sensor(sens))

        if "turns" in conf:
            sens = await sensor.new_sensor(conf["turns"])
            cg.add(var.set_turns_sensor(sens))

        if "percent" in conf:
            sens = await sensor.new_sensor(conf["percent"])
            cg.add(var.set_percent_sensor(sens))

        if "calib_state" in conf:
            sens = await sensor.new_sensor(conf["calib_state"])
            cg.add(var.set_calib_state_sensor(sens))

        if CONF_TORQUE_SWITCH in conf:
            sw = await switch.new_switch(conf[CONF_TORQUE_SWITCH])
            await cg.register_component(sw, conf[CONF_TORQUE_SWITCH])
            cg.add(sw.set_parent(var))
            cg.add(var.set_torque_switch(sw))
