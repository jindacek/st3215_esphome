import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import uart, sensor, switch, text_sensor
from esphome.const import (
    CONF_ID,
    CONF_UART_ID,
    CONF_SPEED,
    UNIT_DEGREES,
    UNIT_PERCENT,
    ICON_ROTATE_RIGHT,
)


DEPENDENCIES = ["text_sensor"]
AUTO_LOAD = ["text_sensor"]


st3215_ns = cg.esphome_ns.namespace("st3215_servo")
St3215Servo = st3215_ns.class_("St3215Servo", cg.PollingComponent, uart.UARTDevice)
St3215TorqueSwitch = st3215_ns.class_("St3215TorqueSwitch", switch.Switch, cg.Component)

St3215RotateAction = st3215_ns.class_("St3215RotateAction", automation.Action)
St3215StopAction = st3215_ns.class_("St3215StopAction", automation.Action)
St3215SetAngleAction = st3215_ns.class_("St3215SetAngleAction", automation.Action)
St3215MoveToTurnsAction = st3215_ns.class_("St3215MoveToTurnsAction", automation.Action)
St3215MoveToPercentAction = st3215_ns.class_("St3215MoveToPercentAction", automation.Action)

CONF_SERVO_ID = "servo_id"
CONF_TURNS_FULL_OPEN = "turns_full_open"
CONF_MAX_ANGLE = "max_angle"
CONF_TORQUE_SWITCH = "torque_switch"
CONF_STATE_TEXT = "state_text"

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

            cv.Optional(CONF_TORQUE_SWITCH): switch.switch_schema(St3215TorqueSwitch),

            # nový textový senzor – stav/hlášky kalibrace
            cv.Optional("state_text"): text_sensor.text_sensor_schema(),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.polling_component_schema("500ms"))
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

    if CONF_TORQUE_SWITCH in config:
        sw = await switch.new_switch(config[CONF_TORQUE_SWITCH])
        await cg.register_component(sw, config[CONF_TORQUE_SWITCH])
        cg.add(sw.set_parent(var))
        cg.add(var.set_torque_switch(sw))

    if CONF_STATE_TEXT in config:
        ts = await text_sensor.new_text_sensor(config[CONF_STATE_TEXT])
        cg.add(var.set_state_text_sensor(ts))

# ----------------- Actions (původní) -----------------

ROTATE_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ID): cv.use_id(St3215Servo),
        cv.Required("direction"): cv.one_of("cw", "ccw", lower=True),
        cv.Optional(CONF_SPEED, default=600): cv.templatable(cv.int_),
    }
)

@automation.register_action("st3215_servo.rotate", St3215RotateAction, ROTATE_SCHEMA)
async def rotate_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id)
    parent = await cg.get_variable(config[CONF_ID])
    cg.add(var.set_parent(parent))
    speed = await cg.templatable(config[CONF_SPEED], args, cg.int_)
    cg.add(var.set_speed(speed))
    cg.add(var.set_cw(config["direction"] == "cw"))
    return var

STOP_SCHEMA = cv.Schema({cv.Required(CONF_ID): cv.use_id(St3215Servo)})

@automation.register_action("st3215_servo.stop", St3215StopAction, STOP_SCHEMA)
async def stop_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id)
    parent = await cg.get_variable(config[CONF_ID])
    cg.add(var.set_parent(parent))
    return var

SET_ANGLE_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ID): cv.use_id(St3215Servo),
        cv.Required("angle"): cv.templatable(cv.float_),
        cv.Optional(CONF_SPEED, default=600): cv.templatable(cv.int_),
    }
)

@automation.register_action("st3215_servo.set_angle", St3215SetAngleAction, SET_ANGLE_SCHEMA)
async def set_angle_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id)
    parent = await cg.get_variable(config[CONF_ID])
    cg.add(var.set_parent(parent))
    angle = await cg.templatable(config["angle"], args, cg.float_)
    speed = await cg.templatable(config[CONF_SPEED], args, cg.int_)
    cg.add(var.set_angle(angle))
    cg.add(var.set_speed(speed))
    return var

MOVE_TURNS_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ID): cv.use_id(St3215Servo),
        cv.Required("turns"): cv.templatable(cv.float_),
        cv.Optional(CONF_SPEED, default=600): cv.templatable(cv.int_),
    }
)

@automation.register_action("st3215_servo.move_to_turns", St3215MoveToTurnsAction, MOVE_TURNS_SCHEMA)
async def move_to_turns_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id)
    parent = await cg.get_variable(config[CONF_ID])
    cg.add(var.set_parent(parent))
    turns = await cg.templatable(config["turns"], args, cg.float_)
    speed = await cg.templatable(config[CONF_SPEED], args, cg.int_)
    cg.add(var.set_turns(turns))
    cg.add(var.set_speed(speed))
    return var

MOVE_PERCENT_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ID): cv.use_id(St3215Servo),
        cv.Required("percent"): cv.templatable(cv.float_),
        cv.Optional(CONF_SPEED, default=600): cv.templatable(cv.int_),
    }
)

@automation.register_action("st3215_servo.move_to_percent", St3215MoveToPercentAction, MOVE_PERCENT_SCHEMA)
async def move_to_percent_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id)
    parent = await cg.get_variable(config[CONF_ID])
    cg.add(var.set_parent(parent))
    percent = await cg.templatable(config["percent"], args, cg.float_)
    speed = await cg.templatable(config[CONF_SPEED], args, cg.int_)
    cg.add(var.set_percent(percent))
    cg.add(var.set_speed(speed))
    return var
