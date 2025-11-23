import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor
from esphome.const import (
    CONF_ID,
    CONF_UART_ID,
    UNIT_DEGREE,
    UNIT_PERCENT,
    ICON_ROTATE_RIGHT,
)

from . import st3215_ns, St3215Servo

CONF_SERVO_ID = "servo_id"
CONF_TURNS_FULL_OPEN = "turns_full_open"
CONF_MAX_ANGLE = "max_angle"

CONF_ANGLE = "angle"
CONF_TURNS = "turns"
CONF_PERCENT = "percent"

St3215SetAngleAction = st3215_ns.class_("St3215SetAngleAction", cg.Action)
St3215RotateAction = st3215_ns.class_("St3215RotateAction", cg.Action)
St3215StopAction = st3215_ns.class_("St3215StopAction", cg.Action)
St3215MoveToTurnsAction = st3215_ns.class_("St3215MoveToTurnsAction", cg.Action)
St3215MoveToPercentAction = st3215_ns.class_("St3215MoveToPercentAction", cg.Action)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(St3215Servo),
        cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
        cv.Optional(CONF_SERVO_ID, default=1): cv.int_range(min=0, max=253),
        cv.Optional(CONF_MAX_ANGLE, default=240.0): cv.float_range(min=1.0, max=3600.0),
        cv.Optional(CONF_TURNS_FULL_OPEN, default=0.0): cv.float_,

        cv.Optional(CONF_ANGLE): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREE,
            icon=ICON_ROTATE_RIGHT,
            accuracy_decimals=1,
        ),
        cv.Optional(CONF_TURNS): sensor.sensor_schema(
            unit_of_measurement="turns",
            icon=ICON_ROTATE_RIGHT,
            accuracy_decimals=3,
        ),
        cv.Optional(CONF_PERCENT): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            icon="mdi:blinds",
            accuracy_decimals=0,
        ),
    }
).extend(cv.polling_component_schema("500ms"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_servo_id(config[CONF_SERVO_ID]))
    cg.add(var.set_max_angle(config[CONF_MAX_ANGLE]))
    cg.add(var.set_turns_full_open(config[CONF_TURNS_FULL_OPEN]))

    if CONF_ANGLE in config:
        s = await sensor.new_sensor(config[CONF_ANGLE])
        cg.add(var.set_angle_sensor(s))
    if CONF_TURNS in config:
        s = await sensor.new_sensor(config[CONF_TURNS])
        cg.add(var.set_turns_sensor(s))
    if CONF_PERCENT in config:
        s = await sensor.new_sensor(config[CONF_PERCENT])
        cg.add(var.set_percent_sensor(s))

@cg.register_action(
    "st3215_servo.set_angle",
    St3215SetAngleAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(St3215Servo),
            cv.Required("angle"): cv.templatable(cv.float_),
            cv.Optional("speed", default=600): cv.templatable(cv.int_),
        }
    ),
)
async def set_angle_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    angle = await cg.templatable(config["angle"], args, cg.float_)
    speed = await cg.templatable(config["speed"], args, cg.int_)
    cg.add(var.set_angle(angle))
    cg.add(var.set_speed(speed))
    return var


@cg.register_action(
    "st3215_servo.rotate",
    St3215RotateAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(St3215Servo),
            cv.Required("direction"): cv.one_of("cw", "ccw", lower=True),
            cv.Optional("speed", default=600): cv.templatable(cv.int_),
        }
    ),
)
async def rotate_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    speed = await cg.templatable(config["speed"], args, cg.int_)
    cg.add(var.set_speed(speed))
    cg.add(var.set_direction(config["direction"] == "cw"))
    return var


@cg.register_action(
    "st3215_servo.stop",
    St3215StopAction,
    cv.Schema({cv.Required(CONF_ID): cv.use_id(St3215Servo)}),
)
async def stop_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, parent)


@cg.register_action(
    "st3215_servo.move_to_turns",
    St3215MoveToTurnsAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(St3215Servo),
            cv.Required("turns"): cv.templatable(cv.float_),
            cv.Optional("speed", default=600): cv.templatable(cv.int_),
        }
    ),
)
async def move_to_turns_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    turns = await cg.templatable(config["turns"], args, cg.float_)
    speed = await cg.templatable(config["speed"], args, cg.int_)
    cg.add(var.set_turns(turns))
    cg.add(var.set_speed(speed))
    return var


@cg.register_action(
    "st3215_servo.move_to_percent",
    St3215MoveToPercentAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(St3215Servo),
            cv.Required("percent"): cv.templatable(cv.float_),
            cv.Optional("speed", default=600): cv.templatable(cv.int_),
        }
    ),
)
async def move_to_percent_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    percent = await cg.templatable(config["percent"], args, cg.float_)
    speed = await cg.templatable(config["speed"], args, cg.int_)
    cg.add(var.set_percent(percent))
    cg.add(var.set_speed(speed))
    return var
