import esphome.codegen as cg

st3215_ns = cg.esphome_ns.namespace("st3215_servo")
St3215Servo = st3215_ns.class_("St3215Servo", cg.PollingComponent, cg.Component)
