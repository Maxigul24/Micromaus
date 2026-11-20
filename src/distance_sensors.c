#include "distance_sensors.h"
#include "platform.h"
#include "vl53l0x.h"

#define TOF_STARTUP_DELAY_MS       15U
#define TOF_POLL_INTERVAL_MS       5U
#define TOF_TIMEOUT_MS             100U

typedef struct {
    tof_sensor_id_t platform_id;
    uint8_t address;
    vl53l0x_t device;
    bool initialized;
    bool valid;
    uint16_t distance_mm;
} distance_sensor_t;

static distance_sensor_t sensors[DISTANCE_SENSOR_COUNT] = {
    { TOF_SENSOR_LEFT,  0x30, { 0, 0 }, false, false, 0 },
    { TOF_SENSOR_FRONT, 0x31, { 0, 0 }, false, false, 0 },
    { TOF_SENSOR_RIGHT, 0x32, { 0, 0 }, false, false, 0 }
};

static uint8_t active_sensor;
static bool measurement_active;
static uint32_t measurement_started_ms;
static uint32_t last_poll_ms;

void distance_sensors_init(void)
{
    for (uint8_t i = 0; i < DISTANCE_SENSOR_COUNT; i++) {
        tof_xshut_set(sensors[i].platform_id, false);
    }

    uart_print("ToF Sensoren:\r\n");
    for (uint8_t i = 0; i < DISTANCE_SENSOR_COUNT; i++) {
        distance_sensor_t *sensor = &sensors[i];
        tof_xshut_set(sensor->platform_id, true);
        delay_ms(TOF_STARTUP_DELAY_MS);
        if (vl53l0x_init(&sensor->device, sensor->address)) {
            sensor->initialized = true;
            uart_print("  Sensor ");
            uart_print_number(i + 1U);
            uart_print(" initialisiert, I2C Adresse ");
            uart_print_number(sensor->address);
            uart_print("\r\n");
        } else {
            tof_xshut_set(sensor->platform_id, false);
            uart_print("  Fehler: Sensor ");
            uart_print_number(i + 1U);
            uart_print(" konnte nicht initialisiert werden\r\n");
        }
    }
}

void distance_sensors_update(uint32_t now)
{
    if (measurement_active) {
        if (now - last_poll_ms < TOF_POLL_INTERVAL_MS) return;
        last_poll_ms = now;

        distance_sensor_t *sensor = &sensors[active_sensor];
        bool ready;
        if (!vl53l0x_read_if_ready(&sensor->device, &sensor->distance_mm,
                                   &ready)) {
            sensor->valid = false;
            sensor->initialized = false;
            tof_xshut_set(sensor->platform_id, false);
            uart_print("Fehler beim Auslesen von ToF Sensor ");
            uart_print_number(active_sensor + 1U);
            uart_print("\r\n");
            measurement_active = false;
            active_sensor = (active_sensor + 1U) % DISTANCE_SENSOR_COUNT;
            return;
        }

        if (ready) {
            sensor->valid = true;
            measurement_active = false;
            active_sensor = (active_sensor + 1U) % DISTANCE_SENSOR_COUNT;
            return;
        }

        if (now - measurement_started_ms >= TOF_TIMEOUT_MS) {
            sensor->valid = false;
            uart_print("Timeout bei ToF Sensor ");
            uart_print_number(active_sensor + 1U);
            uart_print("\r\n");
            measurement_active = false;
            active_sensor = (active_sensor + 1U) % DISTANCE_SENSOR_COUNT;
        }
        return;
    }

    for (uint8_t i = 0; i < DISTANCE_SENSOR_COUNT; i++) {
        uint8_t candidate = active_sensor;
        active_sensor = (active_sensor + 1U) % DISTANCE_SENSOR_COUNT;
        distance_sensor_t *sensor = &sensors[candidate];
        if (!sensor->initialized) continue;

        if (!vl53l0x_start(&sensor->device)) {
            sensor->initialized = false;
            sensor->valid = false;
            tof_xshut_set(sensor->platform_id, false);
            uart_print("Fehler beim Starten von ToF Sensor ");
            uart_print_number(candidate + 1U);
            uart_print("\r\n");
            continue;
        }

        active_sensor = candidate;
        measurement_active = true;
        measurement_started_ms = now;
        last_poll_ms = now;
        return;
    }
}

bool distance_sensors_get(distance_sensor_id_t id, distance_reading_t *reading)
{
    if (reading == 0 || (uint32_t)id >= DISTANCE_SENSOR_COUNT) {
        return false;
    }

    reading->initialized = sensors[id].initialized;
    reading->valid = sensors[id].valid;
    reading->distance_mm = sensors[id].distance_mm;
    return true;
}
