import esphome.codegen as cg
from esphome.components import uart

st3215_ns = cg.esphome_ns.namespace("st3215_servo")

# TADY JE KLÍČOVÉ PŘIDAT uart.UARTDevice !!!
St3215Servo = st3215_ns.class_(
    "St3215Servo",
    cg.PollingComponent,
    uart.UARTDevice,     # <- MUSÍ TAM BÝT
    cg.Component
)
