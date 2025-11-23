import esphome.codegen as cg
from esphome.components import uart

AUTO_LOAD = ["sensor"]          # <- Tohle vynutí přidání sensor komponenty do buildu
DEPENDENCIES = ["uart"]         # <- UART je skutečná závislost

st3215_ns = cg.esphome_ns.namespace("st3215_servo")

St3215Servo = st3215_ns.class_(
    "St3215Servo",
    cg.PollingComponent,
    uart.UARTDevice,
    cg.Component,
)

from .st3215_servo import CONFIG_SCHEMA, to_code  # noqa
