import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import uart, sensor, switch, number, cover
from esphome.const import (
    CONF_ID,
    CONF_UART_ID,
    UNIT_DEGREES,
    UNIT_PERCENT,
)

DEPENDENCIES = ["uart", "sensor", "switch", "number", "cover"]
AUTO_LOAD = ["switch", "cover"]

st3215_ns = cg.esphome_ns.namespace("st3215_servo")

St3215Servo = st3215_ns.class_("St3215Servo", cg.Component, uart.UARTDevice)
St3215TorqueSwitch = st3215_ns.class_("St3215TorqueSwitch", switch.Switch, cg.Component)
St3215Cover = st3215_ns.class_("St3215Cover", cover.Cover, cg.Component)

CONF_SERVO_ID = "servo_id"
CONF_TURNS_FULL_OPEN = "turns_full_open"
CONF_MAX_ANGLE = "max_angle"
CONF_TORQUE_SWITCH = "torque_switch"

CONF_ANGLE = "angle"
CONF_TURNS_SENSOR = "turns"
CONF_PERCENT = "percent"
CONF_CALIB_STATE = "calib_state"

CONF_COVER = "cover"
CONF_OPEN_SPEED = "open_speed"
CONF_CLOSE_SPEED = "close_speed"


# ================= CONFIG SCHEMA =================

TORQUE_SWITCH_SCHEMA = switch.switch_schema(St3215TorqueSwitch)


CONFIG_SCHEMA = uart.UART_DEVICE_SCHEMA.extend(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(St3215Servo),

        cv.Required(CONF_SERVO_ID): cv.uint8_t,
        cv.Optional(CONF_TURNS_FULL_OPEN, default=0.0): cv.float_,
        cv.Optional(CONF_MAX_ANGLE, default=360.0): cv.float_,

        # senzory
        cv.Optional(CONF_ANGLE): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREES,
            accuracy_decimals=2,
        ),
        cv.Optional(CONF_TURNS_SENSOR): sensor.sensor_schema(
            accuracy_decimals=3,
        ),
        cv.Optional(CONF_PERCENT): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=1,
        ),
        cv.Optional(CONF_CALIB_STATE): sensor.sensor_schema(
            accuracy_decimals=0,
        ),

        # torque switch
        cv.Optional(CONF_TORQUE_SWITCH): TORQUE_SWITCH_SCHEMA,

        # COVER – plnohodnotná platforma
        cv.Optional(CONF_COVER): cover.COVER_SCHEMA.extend(
            {
                cv.GenerateID(): cv.declare_id(St3215Cover),
                cv.Required(CONF_OPEN_SPEED): cv.use_id(number.Number),
                cv.Required(CONF_CLOSE_SPEED): cv.use_id(number.Number),
            }
        ),
    }
)


# ================= CODEGEN =================

async def to_code(config):
  # Servo instance
  var = cg.new_Pvariable(config[CONF_ID])
  await cg.register_component(var, config)
  await uart.register_uart_device(var, config)

  cg.add(var.set_servo_id(config[CONF_SERVO_ID]))
  cg.add(var.set_max_angle(config[CONF_MAX_ANGLE]))
  cg.add(var.set_turns_full_open(config[CONF_TURNS_FULL_OPEN]))

  # senzory
  if CONF_ANGLE in config:
      sens = await sensor.new_sensor(config[CONF_ANGLE])
      cg.add(var.set_angle_sensor(sens))

  if CONF_TURNS_SENSOR in config:
      sens = await sensor.new_sensor(config[CONF_TURNS_SENSOR])
      cg.add(var.set_turns_sensor(sens))

  if CONF_PERCENT in config:
      sens = await sensor.new_sensor(config[CONF_PERCENT])
      cg.add(var.set_percent_sensor(sens))

  if CONF_CALIB_STATE in config:
      sens = await sensor.new_sensor(config[CONF_CALIB_STATE])
      cg.add(var.set_calib_state_sensor(sens))

  # torque switch
  if CONF_TORQUE_SWITCH in config:
      sw_conf = config[CONF_TORQUE_SWITCH]
      sw = await switch.new_switch(sw_conf)
      await cg.register_component(sw, sw_conf)
      cg.add(sw.set_parent(var))
      cg.add(var.set_torque_switch(sw))

  # COVER
  if CONF_COVER in config:
      cov_conf = config[CONF_COVER]

      open_num = await cg.get_variable(cov_conf[CONF_OPEN_SPEED])
      close_num = await cg.get_variable(cov_conf[CONF_CLOSE_SPEED])

      c = cg.new_Pvariable(
          cov_conf[CONF_ID],
          var,
          open_num,
          close_num,
      )

      await cg.register_component(c, cov_conf)
      await cover.register_cover(c, cov_conf)
