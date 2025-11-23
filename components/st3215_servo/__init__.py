import esphome.codegen as cg
from esphome.components import uart

st3215_ns = cg.esphome_ns.namespace("st3215_servo")

St3215Servo = st3215_ns.class_(
    "St3215Servo",
    cg.PollingComponent,
    uart.UARTDevice,
    cg.Component,
)

# ESPHome bere CONFIG_SCHEMA pro top-level platformu z __init__.py,
# tak ho sem jen re-exportujeme.
from .st3215_servo import CONFIG_SCHEMA, to_code  # noqa

DEPENDENCIES = ["uart"]
