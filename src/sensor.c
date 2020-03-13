#include <drivers/sensor.h>
#include "sensor.h"

#define TOF_POLLING_TIME_MS 2
#define TOF_TIMEOUT_MS 80
#define TOF_MAX_RANGE 1500

tof_controller_t tof_front;
tof_controller_t tof_down;
void init_sensor(tof_controller_t *sensor, struct device *dev, int range_offset)
{
    sensor->range = 0;
    sensor->dev = dev;
    sensor->time_last_read = 0;
    sensor->range_offset = range_offset;
}

void reset_sensor(tof_controller_t *tof, u16_t target_distance)
{
    tof->integral_e = 0;
    tof->l_error = range_diff_sensor(tof, target_distance);
    tof->derivative1 = 0;
    tof->derivative2 = 0;
}

int range_diff_sensor(tof_controller_t *tof, u16_t target_distance)
{
    return target_distance - tof->range + tof->range_offset;
}

/**
 * get range data from sensor
 * the sensor can timeout TOF_TIMEOUT_MS
 * the sensor has an minimal polling interval of TOF_POLLING_TIME_MS
 */
void fetch_distance_sensor(tof_controller_t *sensor)
{
    struct sensor_value value;
    int ret = sensor_sample_fetch(sensor->dev);

    if ((k_uptime_get() - sensor->time_last_read) <= TOF_POLLING_TIME_MS) {
        return;
    }

    if (ret != -EBUSY && ret != 0) { // unknown error of Sensor
        filterReset(&sensor->filter);
        sensor->range = -EIO;
        return;
    }

    if (ret == -EBUSY) {
        if ((k_uptime_get() - sensor->time_last_read) >= TOF_TIMEOUT_MS) { //reset data on Timeout
            filterReset(&sensor->filter);
            sensor->range = -EIO;
        }
        return;
    }

    if (sensor_channel_get(sensor->dev, SENSOR_CHAN_DISTANCE, &value) == 0) { // no error and new valid value
        sensor->range = sensor_value_to_double(&value) * 1000; // range in mm;
        sensor->time_between_reads = k_uptime_get() - sensor->time_last_read;
        sensor->time_last_read = k_uptime_get();
        sensor->isNew = true;
        if (sensor->range > TOF_MAX_RANGE) {
            sensor->range = TOF_MAX_RANGE;
        }
        sensor->range = filterApply(&sensor->filter, sensor->range);
        return;
    }

    filterReset(&sensor->filter);
    sensor->range = -EIO;
}
