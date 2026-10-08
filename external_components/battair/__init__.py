import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import (
    binary_sensor,
    ble_client,
    esp32_ble_tracker,
    ota,
    sensor,
    text_sensor,
)
from esphome.const import (
    CONF_ID,
    CONF_MAC_ADDRESS,
    DEVICE_CLASS_CONNECTIVITY,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_SIGNAL_STRENGTH,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_CELSIUS,
    UNIT_DECIBEL_MILLIWATT,
    UNIT_PERCENT,
    UNIT_VOLT,
)

CODEOWNERS = ["@alex"]
DEPENDENCIES = ["ble_client", "esp32_ble_tracker"]
AUTO_LOAD = ["sensor", "text_sensor", "binary_sensor"]

battair_ns = cg.esphome_ns.namespace("battair")
BattAirHub = battair_ns.class_(
    "BattAirHub",
    cg.PollingComponent,
    ble_client.BLEClientNode,
    esp32_ble_tracker.ESPBTDeviceListener,
)

CONF_BATTERIES = "batteries"
CONF_BLE_NAME = "ble_name"
CONF_ATTEMPTS = "attempts"
CONF_FOUND_DEVICES = "found_devices"
CONF_SCAN_COUNTER = "scan_counter"
CONF_LOG = "log"
CONF_ONLINE = "online"

UNIT_MILLIVOLT = "mV"
UNIT_MILLIAMP = "mA"

# Порядок совпадает с SensorKind в battair.h.
SENSOR_KEYS = [
    "voltage",
    "cell_1",
    "cell_2",
    "cell_3",
    "cell_4",
    "cell_delta",
    "cycles",
    "temperature",
    "charge_current",
    "discharge_current",
    "level",
    "rssi",
    "errors",
    "outages",
    "overheats",
    "overcharges",
    "overdischarges",
]
# Порядок совпадает с TextKind.
TEXT_KEYS = ["label", "type", "firmware"]


def _cell():
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT,
        icon="mdi:battery",
    )


def _counter(icon):
    return sensor.sensor_schema(
        accuracy_decimals=0,
        state_class=STATE_CLASS_TOTAL_INCREASING,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        icon=icon,
    )


# Токи и уровень публикуются сырыми числами из пакета 0x13:
# единицы не проверены на живом датчике, после первых данных уточнить.
BATTERY_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_BLE_NAME): cv.string,
        # Пустая строка: MAC узнается по ble_name из эфира после загрузки.
        cv.Optional(CONF_MAC_ADDRESS, default=""): cv.Any(
            cv.All(cv.string, cv.Length(max=0)), cv.mac_address
        ),
        cv.Optional("voltage"): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
            icon="mdi:flash-triangle-outline",
        ),
        cv.Optional("cell_1"): _cell(),
        cv.Optional("cell_2"): _cell(),
        cv.Optional("cell_3"): _cell(),
        cv.Optional("cell_4"): _cell(),
        cv.Optional("cell_delta"): sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLIVOLT,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            icon="mdi:scale-balance",
        ),
        cv.Optional("cycles"): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            icon="mdi:battery-sync",
        ),
        cv.Optional("temperature"): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional("charge_current"): sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLIAMP,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional("discharge_current"): sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLIAMP,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional("level"): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            icon="mdi:battery-high",
        ),
        cv.Optional("rssi"): sensor.sensor_schema(
            unit_of_measurement=UNIT_DECIBEL_MILLIWATT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_SIGNAL_STRENGTH,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional("errors"): _counter("mdi:alert-circle-outline"),
        cv.Optional("outages"): _counter("mdi:power-plug-off-outline"),
        cv.Optional("overheats"): _counter("mdi:thermometer-alert"),
        cv.Optional("overcharges"): _counter("mdi:battery-arrow-up"),
        cv.Optional("overdischarges"): _counter("mdi:battery-arrow-down"),
        cv.Optional("label"): text_sensor.text_sensor_schema(
            icon="mdi:battery-heart-variant"
        ),
        cv.Optional("type"): text_sensor.text_sensor_schema(icon="mdi:battery-check"),
        cv.Optional("firmware"): text_sensor.text_sensor_schema(
            icon="mdi:chip", entity_category=ENTITY_CATEGORY_DIAGNOSTIC
        ),
        cv.Optional(CONF_ONLINE): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_CONNECTIVITY,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    }
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(BattAirHub),
            cv.Required(CONF_BATTERIES): cv.All(
                cv.ensure_list(BATTERY_SCHEMA), cv.Length(min=1, max=8)
            ),
            cv.Optional(CONF_ATTEMPTS, default=3): cv.int_range(min=1, max=10),
            cv.Optional(CONF_FOUND_DEVICES): sensor.sensor_schema(
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
                icon="mdi:bluetooth-connect",
            ),
            cv.Optional(CONF_SCAN_COUNTER): sensor.sensor_schema(
                accuracy_decimals=0,
                icon="mdi:counter",
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_LOG): text_sensor.text_sensor_schema(
                icon="mdi:console-line"
            ),
        }
    )
    .extend(cv.polling_component_schema("5min"))
    .extend(ble_client.BLE_CLIENT_SCHEMA)
    .extend(esp32_ble_tracker.ESP_BLE_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await ble_client.register_ble_node(var, config)
    await esp32_ble_tracker.register_ble_device(var, config)
    # Чтобы бросать обход на время OTA, см. on_ota_global_state.
    ota.request_ota_state_listeners()
    cg.add(var.set_attempts(config[CONF_ATTEMPTS]))

    for i, bat in enumerate(config[CONF_BATTERIES]):
        mac = bat[CONF_MAC_ADDRESS]
        mac = 0 if isinstance(mac, str) else mac.as_hex
        cg.add(var.add_battery(bat[CONF_BLE_NAME], mac))
        for kind, key in enumerate(SENSOR_KEYS):
            if key in bat:
                s = await sensor.new_sensor(bat[key])
                cg.add(var.set_sensor(i, kind, s))
        for kind, key in enumerate(TEXT_KEYS):
            if key in bat:
                t = await text_sensor.new_text_sensor(bat[key])
                cg.add(var.set_text(i, kind, t))
        if CONF_ONLINE in bat:
            b = await binary_sensor.new_binary_sensor(bat[CONF_ONLINE])
            cg.add(var.set_online(i, b))

    if CONF_FOUND_DEVICES in config:
        s = await sensor.new_sensor(config[CONF_FOUND_DEVICES])
        cg.add(var.set_found_sensor(s))
    if CONF_SCAN_COUNTER in config:
        s = await sensor.new_sensor(config[CONF_SCAN_COUNTER])
        cg.add(var.set_counter_sensor(s))
    if CONF_LOG in config:
        t = await text_sensor.new_text_sensor(config[CONF_LOG])
        cg.add(var.set_log_sensor(t))
