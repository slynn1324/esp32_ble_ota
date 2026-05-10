import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components.ota import BASE_OTA_SCHEMA, ota_to_code, OTAComponent
from esphome.const import CONF_ID, CONF_PASSWORD

CODEOWNERS = ["@slynn1324"]
DEPENDENCIES = ["esp32_ble_server"]
AUTO_LOAD = ["esp32_ble_server"]

ota_ble_ns = cg.esphome_ns.namespace("esp32_ble_ota")
ESP32BLEOTAComponent = ota_ble_ns.class_("ESP32BLEOTAComponent", OTAComponent)

CONF_SERVICE_UUID = "service_uuid"
CONF_CONTROL_UUID = "control_uuid"
CONF_DATA_UUID    = "data_uuid"
CONF_STATUS_UUID  = "status_uuid"
CONF_CHUNK_TIMEOUT = "chunk_timeout"
CONF_MAX_CHUNK_SIZE = "max_chunk_size"
CONF_PROGRESS_STEP = "progress_step"

# BOTA -> 42 4F 54 41
DEFAULT_SERVICE_UUID = "424F5441-0001-1000-8000-00805F9B34FB"
DEFAULT_CONTROL_UUID = "424F5441-0002-1000-8000-00805F9B34FB"
DEFAULT_DATA_UUID    = "424F5442-0003-1000-8000-00805F9B34FB"
DEFAULT_STATUS_UUID  = "424F5443-0004-1000-8000-00805F9B34FB"

DEFAULT_CHUNK_TIMEOUT_MS = "3s"
DEFAULT_MAX_CHUNK_SIZE = 500;
DEFAULT_PROGRESS_STEP = 5;

CONFIG_SCHEMA = BASE_OTA_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(ESP32BLEOTAComponent),
        cv.Optional(CONF_PASSWORD): cv.All(cv.string, cv.Length(max=64)),
        cv.Optional(CONF_SERVICE_UUID, default=DEFAULT_SERVICE_UUID): cv.uuid,
        cv.Optional(CONF_CONTROL_UUID, default=DEFAULT_CONTROL_UUID): cv.uuid,
        cv.Optional(CONF_DATA_UUID, default=DEFAULT_DATA_UUID): cv.uuid,
        cv.Optional(CONF_STATUS_UUID, default=DEFAULT_STATUS_UUID): cv.uuid,
        cv.Optional(CONF_MAX_CHUNK_SIZE, default=DEFAULT_MAX_CHUNK_SIZE): cv.int_range(min=1, max=500),
        cv.Optional(CONF_CHUNK_TIMEOUT, default=DEFAULT_CHUNK_TIMEOUT_MS): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_PROGRESS_STEP, default=DEFAULT_PROGRESS_STEP): cv.int_range(min=1, max=100),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    if password := config.get(CONF_PASSWORD):
        cg.add(var.set_password(password))
    
    cg.add(var.set_service_uuid(str(config[CONF_SERVICE_UUID])))
    cg.add(var.set_control_uuid(str(config[CONF_CONTROL_UUID])))
    cg.add(var.set_data_uuid(str(config[CONF_DATA_UUID])))
    cg.add(var.set_status_uuid(str(config[CONF_STATUS_UUID])))
    cg.add(var.set_max_chunk_size(config[CONF_MAX_CHUNK_SIZE]))
    cg.add(var.set_chunk_timeout_ms(config[CONF_CHUNK_TIMEOUT]))
    cg.add(var.set_progress_step(config[CONF_PROGRESS_STEP]))

    await cg.register_component(var, config)
    await ota_to_code(var, config)