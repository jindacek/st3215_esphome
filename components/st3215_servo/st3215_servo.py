import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import uart, sensor, switch
from esphome.const import (
    CONF_ID,
    CONF_UART_ID,
    CONF_UPDATE_INTERVAL,
    CONF_SPEED,
)

AUTO_LOAD = ["sensor", "switch"]
DEPENDENCIES = ["uart"]

CONF_SERVO_ID = "servo_id"
CONF_MAX_ANGLE = "max_angle"
CONF_TURNS_FULL_OPEN = "turns_full_open"
CONF_ANGLE = "angle"
CONF_TURNS = "turns"
CONF_PERCENT = "percent"
CONF_TORQUE_SWITCH = "torque_switch"
CONF_DIRECTION = "direction"

st3215_ns = cg.esphome_ns.namespace("st3215_servo")
St3215Servo = st3215_ns.class_("St3215Servo", cg.PollingComponent, uart.UARTDevice)
St3215TorqueSwitch = st3215_ns.class_("St3215TorqueSwitch", switch.Switch, cg.Component)

# --- ESPHome 2025: actions must inherit from automation.Action, not cg.Action ---
St3215RotateAction = st3215_ns.class_("St3215RotateAction", automation.Action)
St3215StopAction = st3215_ns.class_("St3215StopAction", automation.Action)
St3215SetAngleAction = st3215_ns.class_("St3215SetAngleAction", automation.Action)
St3215MoveRelativeAction = st3215_ns.class_("St3215MoveRelativeAction", automation.Action)
St3215MoveToPercentAction = st3215_ns.class_("St3215MoveToPercentAction", automation.Action)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(St3215Servo),
            cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
            cv.Optional(CONF_SERVO_ID, default=1): cv.int_range(min=1, max=253),
            cv.Optional(CONF_MAX_ANGLE, default=240.0): cv.float_,
            cv.Optional(CONF_TURNS_FULL_OPEN, default=0.0): cv.float_,
            cv.Optional(CONF_UPDATE_INTERVAL, default="500ms"): cv.update_interval,
            cv.Optional(CONF_ANGLE): sensor.sensor_schema(unit_of_measurement="°", accuracy_decimals=0),
            cv.Optional(CONF_TURNS): sensor.sensor_schema(unit_of_measurement="turns", accuracy_decimals=3),
            cv.Optional(CONF_PERCENT): sensor.sensor_schema(unit_of_measurement="%", accuracy_decimals=1),
            cv.Optional(CONF_TORQUE_SWITCH): switch.switch_schema(St3215TorqueSwitch),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.polling_component_schema("500ms"))
)

async def to_code(config):
    uart_comp = await cg.get_variable(config[CONF_UART_ID])
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_servo_id(config[CONF_SERVO_ID]))
    cg.add(var.set_max_angle(config[CONF_MAX_ANGLE]))
    cg.add(var.set_turns_full_open(config[CONF_TURNS_FULL_OPEN]))

    if CONF_ANGLE in config:
        sens = await sensor.new_sensor(config[CONF_ANGLE])
        cg.add(var.set_angle_sensor(sens))

    if CONF_TURNS in config:
        sens = await sensor.new_sensor(config[CONF_TURNS])
        cg.add(var.set_turns_sensor(sens))

    if CONF_PERCENT in config:
        sens = await sensor.new_sensor(config[CONF_PERCENT])
        cg.add(var.set_percent_sensor(sens))

    if CONF_TORQUE_SWITCH in config:
        sw = await switch.new_switch(config[CONF_TORQUE_SWITCH], St3215TorqueSwitch)
        await cg.register_component(sw, config[CONF_TORQUE_SWITCH])
        cg.add(var.set_torque_switch(sw))

# -----------------------------
# Actions
# -----------------------------
DIRECTION_ENUM = cv.enum({"cw": True, "ccw": False}, lower=True)

ROTATE_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ID): cv.use_id(St3215Servo),
        cv.Required(CONF_DIRECTION): DIRECTION_ENUM,
        cv.Optional(CONF_SPEED, default=600): cv.int_range(min=0, max=3400),
    }
)

@automation.register_action("st3215_servo.rotate", St3215RotateAction, ROTATE_SCHEMA)
async def rotate_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id)
    cg.add(var.set_parent(parent))
    cg.add(var.set_cw(config[CONF_DIRECTION]))
    cg.add(var.set_speed(config[CONF_SPEED]))
    return var

STOP_SCHEMA = cv.Schema({cv.Required(CONF_ID): cv.use_id(St3215Servo)})

@automation.register_action("st3215_servo.stop", St3215StopAction, STOP_SCHEMA)
async def stop_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id)
    cg.add(var.set_parent(parent))
    return var

SET_ANGLE_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ID): cv.use_id(St3215Servo),
        cv.Required("angle"): cv.float_,
        cv.Optional(CONF_SPEED, default=600): cv.int_range(min=0, max=3400),
    }
)

@automation.register_action("st3215_servo.set_angle", St3215SetAngleAction, SET_ANGLE_SCHEMA)
async def set_angle_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id)
    cg.add(var.set_parent(parent))
    cg.add(var.set_angle(config["angle"]))
    cg.add(var.set_speed(config[CONF_SPEED]))
    return var

MOVE_REL_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ID): cv.use_id(St3215Servo),
        cv.Required("turns"): cv.float_,
        cv.Optional(CONF_SPEED, default=600): cv.int_range(min=0, max=3400),
    }
)

@automation.register_action("st3215_servo.move_relative", St3215MoveRelativeAction, MOVE_REL_SCHEMA)
async def move_rel_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id)
    cg.add(var.set_parent(parent))
    cg.add(var.set_turns(config["turns"]))
    cg.add(var.set_speed(config[CONF_SPEED]))
    return var

MOVE_PCT_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ID): cv.use_id(St3215Servo),
        cv.Required("percent"): cv.float_range(min=0, max=100),
        cv.Optional(CONF_SPEED, default=600): cv.int_range(min=0, max=3400),
    }
)

@automation.register_action("st3215_servo.move_to_percent", St3215MoveToPercentAction, MOVE_PCT_SCHEMA)
async def move_pct_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id)
    cg.add(var.set_parent(parent))
    cg.add(var.set_percent(config["percent"]))
    cg.add(var.set_speed(config[CONF_SPEED]))
    return var
