import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import ble_client, sensor
from esphome.const import (
    CONF_ID,
    CONF_CURRENT,
    CONF_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_BATTERY,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_VOLT,
    UNIT_AMPERE,
    UNIT_CELSIUS,
    UNIT_PERCENT,
    UNIT_EMPTY,
)

CODEOWNERS = ["@alex"]
DEPENDENCIES = ["ble_client"]
AUTO_LOAD = ["sensor"]

deye_bms_ns = cg.esphome_ns.namespace("deye_bms")
DeyeBMS = deye_bms_ns.class_(
    "DeyeBMS", cg.PollingComponent, ble_client.BLEClientNode
)

CONF_BLE_NAME = "ble_name"
CONF_CELLS = "cells"
CONF_CELL_MAX = "cell_max_voltage"
CONF_CELL_MIN = "cell_min_voltage"
CONF_CELL_DIFF = "cell_delta_voltage"
CONF_CELL_MAX_NO = "cell_max_number"
CONF_CELL_MIN_NO = "cell_min_number"
CONF_TOTAL_VOLTAGE = "total_voltage"
CONF_SOC = "state_of_charge"
CONF_SOH = "state_of_health"
CONF_TEMP_CELL = "temperature_cell"
CONF_TEMP_MOS = "temperature_mos"
CONF_TEMP_ENV = "temperature_env"
CONF_CHARGE_CURRENT_LIMIT = "charge_current_limit"
CONF_DISCHARGE_CURRENT_LIMIT = "discharge_current_limit"
CONF_CHARGE_VOLTAGE_LIMIT = "charge_voltage_limit"
CONF_DISCHARGE_VOLTAGE_LIMIT = "discharge_voltage_limit"
CONF_FULL_CAPACITY = "full_capacity"
CONF_CYCLES = "cycles"

UNIT_MILLIVOLT = "mV"
UNIT_AMPERE_HOUR = "Ah"


def cell_sensor():
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT,
    )


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(DeyeBMS),
            cv.Required(CONF_BLE_NAME): cv.string,
            cv.Optional(CONF_CELLS): cv.All(
                cv.ensure_list(cell_sensor()), cv.Length(min=1, max=16)
            ),
            cv.Optional(CONF_CELL_MAX): cell_sensor(),
            cv.Optional(CONF_CELL_MIN): cell_sensor(),
            cv.Optional(CONF_CELL_DIFF): sensor.sensor_schema(
                unit_of_measurement=UNIT_MILLIVOLT,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CELL_MAX_NO): sensor.sensor_schema(
                accuracy_decimals=0, state_class=STATE_CLASS_MEASUREMENT
            ),
            cv.Optional(CONF_CELL_MIN_NO): sensor.sensor_schema(
                accuracy_decimals=0, state_class=STATE_CLASS_MEASUREMENT
            ),
            cv.Optional(CONF_TOTAL_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CURRENT): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_CURRENT,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_SOC): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_BATTERY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_SOH): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TEMP_CELL): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TEMP_MOS): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TEMP_ENV): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CHARGE_CURRENT_LIMIT): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_CURRENT,
            ),
            cv.Optional(CONF_DISCHARGE_CURRENT_LIMIT): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_CURRENT,
            ),
            cv.Optional(CONF_CHARGE_VOLTAGE_LIMIT): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_VOLTAGE,
            ),
            cv.Optional(CONF_DISCHARGE_VOLTAGE_LIMIT): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_VOLTAGE,
            ),
            cv.Optional(CONF_FULL_CAPACITY): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE_HOUR,
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CYCLES): sensor.sensor_schema(
                accuracy_decimals=0,
                state_class=STATE_CLASS_TOTAL_INCREASING,
            ),
        }
    )
    .extend(cv.polling_component_schema("10s"))
    .extend(ble_client.BLE_CLIENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await ble_client.register_ble_node(var, config)

    cg.add(var.set_ble_name(config[CONF_BLE_NAME]))

    if CONF_CELLS in config:
        for i, conf in enumerate(config[CONF_CELLS]):
            s = await sensor.new_sensor(conf)
            cg.add(var.set_cell(i, s))

    for key, setter in [
        (CONF_CELL_MAX, var.set_cell_max),
        (CONF_CELL_MIN, var.set_cell_min),
        (CONF_CELL_DIFF, var.set_cell_diff),
        (CONF_CELL_MAX_NO, var.set_cell_max_no),
        (CONF_CELL_MIN_NO, var.set_cell_min_no),
        (CONF_TOTAL_VOLTAGE, var.set_total_voltage),
        (CONF_CURRENT, var.set_current),
        (CONF_SOC, var.set_soc),
        (CONF_SOH, var.set_soh),
        (CONF_TEMP_CELL, var.set_temp_cell),
        (CONF_TEMP_MOS, var.set_temp_mos),
        (CONF_TEMP_ENV, var.set_temp_env),
        (CONF_CHARGE_CURRENT_LIMIT, var.set_charge_current_limit),
        (CONF_DISCHARGE_CURRENT_LIMIT, var.set_discharge_current_limit),
        (CONF_CHARGE_VOLTAGE_LIMIT, var.set_charge_voltage_limit),
        (CONF_DISCHARGE_VOLTAGE_LIMIT, var.set_discharge_voltage_limit),
        (CONF_FULL_CAPACITY, var.set_full_capacity),
        (CONF_CYCLES, var.set_cycles),
    ]:
        if key in config:
            s = await sensor.new_sensor(config[key])
            cg.add(setter(s))
