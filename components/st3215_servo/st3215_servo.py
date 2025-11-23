import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, switch
from esphome import automation
from esphome.const import CONF_ID, CONF_UART_ID, CONF_SPEED

DEPENDENCIES = ["uart", "sensor", "switch"]
AUTO_LOAD = ["switch"]

st3215_ns = cg.esphome_ns.namespace("st3215_servo")
St3215Servo = st3215_ns.class_("St3215Servo", cg.Component, uart.UARTDevice)
St3215TorqueSwitch = st3215_ns.class_("St3215TorqueSwitch", switch.Switch, cg.Component)

# Action classes (non-templated)
St3215RotateAction = st3215_ns.class_("St3215RotateAction", automation.Action)
St3215StopAction = st3215_ns.class_("St3215StopAction", automation.Action)
St3215SetAngleAction = st3215_ns.class_("St3215SetAngleAction", automation.Action)
St3215MoveRelativeAction = st3215_ns.class_("St3215MoveRelativeAction", automation.Action)
St3215MoveToPercentAction = st3215_ns.class_("St3215MoveToPercentAction", automation.Action)

CONF_SERVO_ID = "servo_id"
CONF_DIRECTION = "direction"
CONF_TURNS_FULL_OPEN = "turns_full_open"
CONF_TORQUE_SWITCH = "torque_switch"
CONF_ANGLE = "angle"
CONF_TURNS = "turns"
CONF_PERCENT = "percent"
CONF_TORQUE = "torque"

DIRECTIONS = {"cw": True, "ccw": False}

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(St3215Servo),
            cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
            cv.Required(CONF_SERVO_ID): cv.int_range(min=0, max=253),
            cv.Required(CONF_TURNS_FULL_OPEN): cv.float_,

            cv.Optional(CONF_ANGLE): sensor.sensor_schema(),
            cv.Optional(CONF_TURNS): sensor.sensor_schema(),
            cv.Optional(CONF_PERCENT): sensor.sensor_schema(),
            cv.Optional(CONF_TORQUE): sensor.sensor_schema(),

            cv.Optional(CONF_TORQUE_SWITCH): switch.switch_schema(St3215TorqueSwitch),
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
        s = await sensor.new_sensor(config[CONF_TORQUE])
        cg.add(var.set_torque_sensor(s))

    if CONF_TORQUE_SWITCH in config:
        sw = cg.new_Pvariable(config[CONF_TORQUE_SWITCH][CONF_ID])
        await cg.register_component(sw, config[CONF_TORQUE_SWITCH])
        await switch.register_switch(sw, config[CONF_TORQUE_SWITCH])
        cg.add(var.set_torque_switch(sw))
        cg.add(sw.set_parent(var))


# ---------------- ROTATE ----------------
ROTATE_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ID): cv.use_id(St3215Servo),
        cv.Required(CONF_DIRECTION): cv.one_of(*DIRECTIONS, lower=True),
        cv.Required(CONF_SPEED): cv.int_range(min=10, max=1000),
    }
)

@automation.register_action("st3215_servo.rotate", St3215RotateAction, ROTATE_SCHEMA)
async def rotate_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id)
    par = await cg.get_variable(config[CONF_ID])
    cg.add(var.set_parent(par))
    cg.add(var.set_cw(DIRECTIONS[config[CONF_DIRECTION]]))
    cg.add(var.set_speed(config[CONF_SPEED]))
    return var


# ---------------- STOP ----------------
STOP_SCHEMA = cv.Schema({cv.Required(CONF_ID): cv.use_id(St3215Servo)})

@automation.register_action("st3215_servo.stop", St3215StopAction, STOP_SCHEMA)
async def stop_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id)
    par = await cg.get_variable(config[CONF_ID])
    cg.add(var.set_parent(par))
    return var


# ---------------- SET ANGLE ----------------
SET_ANGLE_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ID): cv.use_id(St3215Servo),
        cv.Required("angle"): cv.float_,
        cv.Optional(CONF_SPEED, default=600): cv.int_range(min=10, max=1000),
    }
)

@automation.register_action("st3215_servo.set_angle", St3215SetAngleAction, SET_ANGLE_SCHEMA)
async def set_angle_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id)
    par = await cg.get_variable(config[CONF_ID])
    cg.add(var.set_parent(par))
    cg.add(var.set_angle(config["angle"]))
    cg.add(var.set_speed(config[CONF_SPEED]))
    return var


# ---------------- MOVE RELATIVE (turns) ----------------
MOVE_REL_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ID): cv.use_id(St3215Servo),
        cv.Required("turns"): cv.float_,
        cv.Optional(CONF_SPEED, default=600): cv.int_range(min=10, max=1000),
    }
)

@automation.register_action("st3215_servo.move_relative", St3215MoveRelativeAction, MOVE_REL_SCHEMA)
async def move_relative_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id)
    par = await cg.get_variable(config[CONF_ID])
    cg.add(var.set_parent(par))
    cg.add(var.set_turns(config["turns"]))
    cg.add(var.set_speed(config[CONF_SPEED]))
    return var


# ---------------- MOVE TO PERCENT ----------------
MOVE_PCT_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ID): cv.use_id(St3215Servo),
        cv.Required("percent"): cv.float_range(min=0, max=100),
        cv.Optional(CONF_SPEED, default=600): cv.int_range(min=10, max=1000),
    }
)

@automation.register_action("st3215_servo.move_to_percent", St3215MoveToPercentAction, MOVE_PCT_SCHEMA)
async def move_to_percent_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id)
    par = await cg.get_variable(config[CONF_ID])
    cg.add(var.set_parent(par))
    cg.add(var.set_percent(config["percent"]))
    cg.add(var.set_speed(config[CONF_SPEED]))
    return var
