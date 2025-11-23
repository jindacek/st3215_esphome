import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, switch
from esphome.const import (
    CONF_ID,
    CONF_UART_ID,
)

CODEOWNERS = ["@jindacek"]
AUTO_LOAD = ["uart", "sensor", "switch", "button"]
DEPENDENCIES = ["uart"]

st3215_ns = cg.esphome_ns.namespace("st3215_servo")
St3215Servo = st3215_ns.class_("St3215Servo", cg.PollingComponent, uart.UARTDevice)

CONF_SERVO_ID = "servo_id"
CONF_MAX_ANGLE = "max_angle"
CONF_TURNS_FULL_OPEN = "turns_full_open"

CONF_ANGLE = "angle"
CONF_TURNS = "turns"
CONF_PERCENT = "percent"

CONF_TORQUE = "torque"

UNIT_TURNS = "turns"

NUM_SENSOR_SCHEMA = sensor.sensor_schema(
    accuracy_decimals=3,
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(St3215Servo),
            cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
            cv.Optional(CONF_SERVO_ID, default=1): cv.int_range(min=0, max=253),
            cv.Optional(CONF_MAX_ANGLE, default=240.0): cv.float_range(min=1.0, max=360.0),
            cv.Optional(CONF_TURNS_FULL_OPEN, default=0.0): cv.float_,
            cv.Optional(CONF_ANGLE): sensor.sensor_schema(
                unit_of_measurement="°",
                accuracy_decimals=1,
                icon="mdi:rotate-right",
            ),
            cv.Optional(CONF_TURNS): sensor.sensor_schema(
                unit_of_measurement=UNIT_TURNS,
                accuracy_decimals=3,
                icon="mdi:rotate-right",
            ),
            cv.Optional(CONF_PERCENT): sensor.sensor_schema(
                unit_of_measurement="%",
                accuracy_decimals=0,
                icon="mdi:blinds",
            ),
            cv.Optional(CONF_TORQUE): switch.switch_schema(
                icon="mdi:arm-flex",
            ),
        }
    )
    .extend(cv.polling_component_schema("500ms"))
)

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

    if CONF_TORQUE in config:
        sw = await switch.new_switch(config[CONF_TORQUE])
        cg.add(var.set_torque_switch(sw))

# Actions (YAML)
CONF_DIRECTION = "direction"
CONF_SPEED = "speed"
CONF_TURNS_DELTA = "turns"
CONF_PERCENT_TARGET = "percent"
CONF_ANGLE_TARGET = "angle"

St3215RotateAction = st3215_ns.class_("St3215RotateAction", cg.Action)
St3215StopAction = st3215_ns.class_("St3215StopAction", cg.Action)
St3215MoveRelativeAction = st3215_ns.class_("St3215MoveRelativeAction", cg.Action)
St3215SetAngleAction = st3215_ns.class_("St3215SetAngleAction", cg.Action)
St3215MoveToPercentAction = st3215_ns.class_("St3215MoveToPercentAction", cg.Action)

@cg.register_action(
    "st3215_servo.rotate",
    St3215RotateAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(St3215Servo),
            cv.Required(CONF_DIRECTION): cv.one_of("cw", "ccw", lower=True),
            cv.Optional(CONF_SPEED, default=600): cv.int_range(min=0, max=3400),
        }
    ),
)
async def st3215_rotate_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    cg.add(var.set_cw(config[CONF_DIRECTION] == "cw"))
    cg.add(var.set_speed(config[CONF_SPEED]))
    return var

@cg.register_action(
    "st3215_servo.stop",
    St3215StopAction,
    cv.Schema({cv.Required(CONF_ID): cv.use_id(St3215Servo)}),
)
async def st3215_stop_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, parent)

@cg.register_action(
    "st3215_servo.move_relative",
    St3215MoveRelativeAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(St3215Servo),
            cv.Required(CONF_TURNS_DELTA): cv.float_,
            cv.Optional(CONF_SPEED, default=600): cv.int_range(min=0, max=3400),
        }
    ),
)
async def st3215_move_relative_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    cg.add(var.set_turns(config[CONF_TURNS_DELTA]))
    cg.add(var.set_speed(config[CONF_SPEED]))
    return var

@cg.register_action(
    "st3215_servo.set_angle",
    St3215SetAngleAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(St3215Servo),
            cv.Required(CONF_ANGLE_TARGET): cv.float_,
            cv.Optional(CONF_SPEED, default=600): cv.int_range(min=0, max=3400),
        }
    ),
)
async def st3215_set_angle_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    cg.add(var.set_angle(config[CONF_ANGLE_TARGET]))
    cg.add(var.set_speed(config[CONF_SPEED]))
    return var

@cg.register_action(
    "st3215_servo.move_to_percent",
    St3215MoveToPercentAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(St3215Servo),
            cv.Required(CONF_PERCENT_TARGET): cv.float_range(min=0.0, max=100.0),
            cv.Optional(CONF_SPEED, default=600): cv.int_range(min=0, max=3400),
        }
    ),
)
async def st3215_move_to_percent_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    cg.add(var.set_percent(config[CONF_PERCENT_TARGET]))
    cg.add(var.set_speed(config[CONF_SPEED]))
    return var
