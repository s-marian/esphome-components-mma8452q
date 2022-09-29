import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID,
    CONF_TEMPERATURE,
    DEVICE_CLASS_TEMPERATURE,
    ICON_BRIEFCASE_DOWNLOAD,
    STATE_CLASS_MEASUREMENT,
    UNIT_METER_PER_SECOND_SQUARED,
    ICON_SCREEN_ROTATION,
    UNIT_DEGREE_PER_SECOND,
    UNIT_DEGREES,
    UNIT_CELSIUS,
)

DEPENDENCIES = ["i2c"]

CONF_ACCEL_X = "accel_x"
CONF_ACCEL_Y = "accel_y"
CONF_ACCEL_Z = "accel_z"
CONF_ROLL    = "roll"
CONF_PITCH   = "pitch"
CONF_ALWAYS_DOWN = "always_look_down"
CONF_ORIENTATION = "orientation"

mma8452q_ns = cg.esphome_ns.namespace("mma8452q")
MMA8452QComponent = mma8452q_ns.class_(
    "MMA8452QComponent", cg.PollingComponent, i2c.I2CDevice
)

accel_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_METER_PER_SECOND_SQUARED,
    icon=ICON_BRIEFCASE_DOWNLOAD,
    accuracy_decimals=2,
    state_class=STATE_CLASS_MEASUREMENT,
)


angle_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_DEGREES,
    icon=ICON_BRIEFCASE_DOWNLOAD,
    accuracy_decimals=1,
    state_class=STATE_CLASS_MEASUREMENT,
)


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MMA8452QComponent),
            cv.Optional(CONF_ACCEL_X): accel_schema,
            cv.Optional(CONF_ACCEL_Y): accel_schema,
            cv.Optional(CONF_ACCEL_Z): accel_schema,
            cv.Optional(CONF_ALWAYS_DOWN): cv.boolean,
            cv.Optional(CONF_ORIENTATION): cv.one_of("xy", "yx"),
            cv.Optional(CONF_ROLL): angle_schema,
            cv.Optional(CONF_PITCH): angle_schema
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x68))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    for d in ["x", "y", "z"]:
        accel_key = f"accel_{d}"
        if accel_key in config:
            sens = await sensor.new_sensor(config[accel_key])
            cg.add(getattr(var, f"set_accel_{d}_sensor")(sens))

    roll_key = f"roll"
    if roll_key in config:
        sens = await sensor.new_sensor(config[roll_key])
        cg.add(getattr(var, f"set_roll_sensor")(sens))
            


    pitch_key = f"pitch"
    if pitch_key in config:
        sens = await sensor.new_sensor(config[pitch_key])
        cg.add(getattr(var, f"set_pitch_sensor")(sens))

    if CONF_ALWAYS_DOWN in config:
        cg.add(var.set_always_look_down(config[CONF_ALWAYS_DOWN]))

    if CONF_ORIENTATION in config:
        cg.add(var.set_orientation(config[CONF_ORIENTATION]))
