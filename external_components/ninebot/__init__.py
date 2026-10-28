import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import (
    binary_sensor,
    ble_client,
    esp32_ble_tracker,
    sensor,
    text_sensor,
)
from esphome.const import (
    CONF_ID,
    CONF_MAC_ADDRESS,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_CONNECTIVITY,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_DISTANCE,
    DEVICE_CLASS_DURATION,
    DEVICE_CLASS_SIGNAL_STRENGTH,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_AMPERE,
    UNIT_CELSIUS,
    UNIT_DECIBEL_MILLIWATT,
    UNIT_KILOMETER,
    UNIT_MINUTE,
    UNIT_PERCENT,
    UNIT_VOLT,
)

CODEOWNERS = ["@alex"]
DEPENDENCIES = ["ble_client", "esp32_ble_tracker"]
AUTO_LOAD = ["sensor", "text_sensor", "binary_sensor"]

ninebot_ns = cg.esphome_ns.namespace("ninebot")
NinebotHub = ninebot_ns.class_(
    "NinebotHub",
    cg.PollingComponent,
    ble_client.BLEClientNode,
    esp32_ble_tracker.ESPBTDeviceListener,
)

CONF_BLE_NAME = "ble_name"
CONF_MAC_SUFFIX = "mac_suffix"
CONF_PASSWORD = "password"

# Порядок строго совпадает с SensorKind в ninebot.h.
SENSOR_KEYS = [
    "battery", "range", "odometer", "body_temperature", "error_code", "warning_code",
    "bms_soc", "voltage", "current", "bms_temperature", "cell_min", "cell_max",
    "cell_delta", "cycles", "remaining_capacity", "time_to_full", "rssi",
]
# Совпадает с TextKind.
TEXT_KEYS = ["status", "serial", "firmware", "log", "pair_password"]
# Совпадает с BinaryKind.
BINARY_KEYS = ["online", "paired", "bms_awake"]


def _pct(icon):
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_PERCENT, accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT, icon=icon,
    )


def _volt():
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT, accuracy_decimals=2,
        device_class=DEVICE_CLASS_VOLTAGE, state_class=STATE_CLASS_MEASUREMENT,
    )


SENSOR_SCHEMAS = {
    "battery": sensor.sensor_schema(
        unit_of_measurement=UNIT_PERCENT, accuracy_decimals=0,
        device_class=DEVICE_CLASS_BATTERY, state_class=STATE_CLASS_MEASUREMENT),
    "bms_soc": _pct("mdi:battery-charging"),
    "range": sensor.sensor_schema(
        unit_of_measurement=UNIT_KILOMETER, accuracy_decimals=1,
        device_class=DEVICE_CLASS_DISTANCE, state_class=STATE_CLASS_MEASUREMENT,
        icon="mdi:map-marker-distance"),
    "odometer": sensor.sensor_schema(
        unit_of_measurement=UNIT_KILOMETER, accuracy_decimals=1,
        device_class=DEVICE_CLASS_DISTANCE, state_class=STATE_CLASS_TOTAL_INCREASING,
        icon="mdi:counter"),
    "body_temperature": sensor.sensor_schema(
        unit_of_measurement=UNIT_CELSIUS, accuracy_decimals=1,
        device_class=DEVICE_CLASS_TEMPERATURE, state_class=STATE_CLASS_MEASUREMENT),
    "bms_temperature": sensor.sensor_schema(
        unit_of_measurement=UNIT_CELSIUS, accuracy_decimals=1,
        device_class=DEVICE_CLASS_TEMPERATURE, state_class=STATE_CLASS_MEASUREMENT),
    "error_code": sensor.sensor_schema(
        accuracy_decimals=0, icon="mdi:alert-circle-outline",
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    "warning_code": sensor.sensor_schema(
        accuracy_decimals=0, icon="mdi:alert-outline",
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    "voltage": _volt(),
    "current": sensor.sensor_schema(
        unit_of_measurement=UNIT_AMPERE, accuracy_decimals=2,
        device_class=DEVICE_CLASS_CURRENT, state_class=STATE_CLASS_MEASUREMENT),
    "cell_min": _volt(),
    "cell_max": _volt(),
    "cell_delta": sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT, accuracy_decimals=3,
        device_class=DEVICE_CLASS_VOLTAGE, state_class=STATE_CLASS_MEASUREMENT,
        icon="mdi:scale-balance"),
    "cycles": sensor.sensor_schema(
        accuracy_decimals=0, state_class=STATE_CLASS_TOTAL_INCREASING,
        icon="mdi:battery-sync", entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    "remaining_capacity": sensor.sensor_schema(
        accuracy_decimals=0, icon="mdi:battery-70", state_class=STATE_CLASS_MEASUREMENT),
    "time_to_full": sensor.sensor_schema(
        unit_of_measurement=UNIT_MINUTE, accuracy_decimals=0,
        device_class=DEVICE_CLASS_DURATION, icon="mdi:timer-sand"),
    "rssi": sensor.sensor_schema(
        unit_of_measurement=UNIT_DECIBEL_MILLIWATT, accuracy_decimals=0,
        device_class=DEVICE_CLASS_SIGNAL_STRENGTH, state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
}

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(NinebotHub),
            cv.Optional(CONF_MAC_ADDRESS): cv.mac_address,
            cv.Optional(CONF_MAC_SUFFIX): cv.hex_uint16_t,
            cv.Optional(CONF_BLE_NAME): cv.string,
            cv.Optional(CONF_PASSWORD): cv.Any(
                cv.All(cv.string, cv.Length(max=0)),
                cv.All(cv.string, cv.Length(min=32, max=32)),
            ),
            **{cv.Optional(k): SENSOR_SCHEMAS[k] for k in SENSOR_SCHEMAS},
            cv.Optional("status"): text_sensor.text_sensor_schema(icon="mdi:information-outline"),
            cv.Optional("serial"): text_sensor.text_sensor_schema(
                icon="mdi:identifier", entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
            cv.Optional("firmware"): text_sensor.text_sensor_schema(
                icon="mdi:chip", entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
            cv.Optional("log"): text_sensor.text_sensor_schema(icon="mdi:console-line"),
            cv.Optional("pair_password"): text_sensor.text_sensor_schema(
                icon="mdi:key", entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
            cv.Optional("online"): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_CONNECTIVITY, entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
            cv.Optional("paired"): binary_sensor.binary_sensor_schema(
                icon="mdi:link-variant", entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
            cv.Optional("bms_awake"): binary_sensor.binary_sensor_schema(
                icon="mdi:sleep-off", entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
        }
    )
    .extend(cv.polling_component_schema("5min"))
    .extend(ble_client.BLE_CLIENT_SCHEMA)
    .extend(esp32_ble_tracker.ESP_BLE_DEVICE_SCHEMA)
)


async def to_code(config):
    cg.add_global(cg.RawExpression('#include "esphome/components/ninebot/automation.h"'))
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await ble_client.register_ble_node(var, config)
    await esp32_ble_tracker.register_ble_device(var, config)

    if CONF_MAC_ADDRESS in config:
        cg.add(var.set_mac(config[CONF_MAC_ADDRESS].as_hex))
    if CONF_MAC_SUFFIX in config:
        cg.add(var.set_mac_suffix(config[CONF_MAC_SUFFIX]))
    if CONF_BLE_NAME in config:
        cg.add(var.set_ble_name(config[CONF_BLE_NAME]))
    if CONF_PASSWORD in config and len(config[CONF_PASSWORD]) == 32:
        cg.add(var.set_password(config[CONF_PASSWORD]))

    for kind, key in enumerate(SENSOR_KEYS):
        if key in config:
            cg.add(var.set_sensor(kind, await sensor.new_sensor(config[key])))
    for kind, key in enumerate(TEXT_KEYS):
        if key in config:
            cg.add(var.set_text(kind, await text_sensor.new_text_sensor(config[key])))
    for kind, key in enumerate(BINARY_KEYS):
        if key in config:
            cg.add(var.set_binary(kind, await binary_sensor.new_binary_sensor(config[key])))


# Действия для кнопок/автоматизаций HA.
PairAction = ninebot_ns.class_("PairAction", automation.Action)
ForgetAction = ninebot_ns.class_("ForgetAction", automation.Action)
PollAction = ninebot_ns.class_("PollAction", automation.Action)
PauseAction = ninebot_ns.class_("PauseAction", automation.Action)

ACTION_SCHEMA = automation.maybe_simple_id({cv.GenerateID(): cv.use_id(NinebotHub)})


@automation.register_action("ninebot.pair", PairAction, ACTION_SCHEMA, synchronous=True)
async def pair_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action("ninebot.forget", ForgetAction, ACTION_SCHEMA, synchronous=True)
async def forget_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


@automation.register_action("ninebot.poll", PollAction, ACTION_SCHEMA, synchronous=True)
async def poll_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, paren)


PAUSE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(NinebotHub),
        cv.Required("paused"): cv.templatable(cv.boolean),
    }
)


@automation.register_action("ninebot.pause", PauseAction, PAUSE_SCHEMA, synchronous=True)
async def pause_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    tmpl = await cg.templatable(config["paused"], args, bool)
    cg.add(var.set_paused(tmpl))
    return var
