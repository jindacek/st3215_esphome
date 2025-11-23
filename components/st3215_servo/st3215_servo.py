from esphome.components import sensor
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome import automation
from esphome.const import (
    CONF_ID,
    CONF_UART_ID,
    CONF_SPEED,
)

st3215_ns = cg.esphome_ns.namespace("st3215_servo")
St3215Servo = st3215_ns.class_("St3215Servo", cg.Component, uart.UARTDevice)

CONF_SERVO_ID = "servo_id"
CONF_DIRECTION = "direction"
CONF_TURNS_FULL_OPEN = "turns_full_open"

DIRECTIONS = {
    "cw": 0,
    "ccw": 1,
}

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(St3215Servo),
            cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
            cv.Required(CONF_SERVO_ID): cv.int_range(min=0, max=100),
            cv.Required(CONF_TURNS_FULL_OPEN): cv.float_,

            cv.Optional("angle"): sensor.sensor_schema(),
            cv.Optional("turns"): sensor.sensor_schema(),
            cv.Optional("percent"): sensor.sensor_schema(),
            cv.Optional("torque"): sensor.sensor_schema(),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)



async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_servo_id(config[CONF_SERVO_ID]))
    cg.add(var.set_turns_full_open(config[CONF_TURNS_FULL_OPEN]))

    # Sensors
    if "angle" in config:
        sens = await sensor.new_sensor(config["angle"])
        cg.add(var.set_angle_sensor(sens))

    if "turns" in config:
        sens = await sensor.new_sensor(config["turns"])
        cg.add(var.set_turns_sensor(sens))

    if "percent" in config:
        sens = await sensor.new_sensor(config["percent"])
        cg.add(var.set_percent_sensor(sens))

    if "torque" in config:
        sens = await sensor.new_sensor(config["torque"])
        cg.add(var.set_torque_sensor(sens))



# -----------------------------
# ROTATE ACTION
# -----------------------------

St3215RotateAction = st3215_ns.class_("St3215RotateAction", automation.Action)

ROTATE_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ID): cv.use_id(St3215Servo),
        cv.Required(CONF_DIRECTION): cv.one_of(*DIRECTIONS, lower=True),
        cv.Required(CONF_SPEED): cv.int_range(min=10, max=1000),
    }
)


@automation.register_action(
    "st3215_servo.rotate",
    St3215RotateAction,
    ROTATE_SCHEMA,
)
async def rotate_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id)
    par = await cg.get_variable(config[CONF_ID])
    cg.add(var.set_parent(par))
    cg.add(var.set_direction(DIRECTIONS[config[CONF_DIRECTION]]))
    cg.add(var.set_speed(config[CONF_SPEED]))
    return var


# -----------------------------
# STOP ACTION
# -----------------------------

St3215StopAction = st3215_ns.class_("St3215StopAction", automation.Action)

STOP_SCHEMA = cv.Schema(
    {cv.Required(CONF_ID): cv.use_id(St3215Servo)}
)


@automation.register_action(
    "st3215_servo.stop",
    St3215StopAction,
    STOP_SCHEMA,
)
async def stop_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id)
    par = await cg.get_variable(config[CONF_ID])
    cg.add(var.set_parent(par))
    return var

