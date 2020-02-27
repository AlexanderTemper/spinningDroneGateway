/* vl53l0x.c - Driver for ST VL53L0X time of flight sensor */

/*
 * Copyright (c) 2017 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <kernel.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <init.h>
#include <drivers/gpio.h>
#include <sys/__assert.h>
#include <zephyr/types.h>
#include <device.h>
#include <logging/log.h>

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

LOG_MODULE_REGISTER( VL53L0X, CONFIG_SENSOR_LOG_LEVEL);

/* All the values used in this driver are coming from ST datasheet and examples.
 * It can be found here:
 *   http://www.st.com/en/embedded-software/stsw-img005.html
 * There are also examples of use in the L4 cube FW:
 *   http://www.st.com/en/embedded-software/stm32cubel4.html
 */
#define VL53L0X_REG_WHO_AM_I   0xC0
#define VL53L0X_CHIP_ID        0xEEAA
#define VL53L0X_SETUP_SIGNAL_LIMIT         (0.1*65536)
#define VL53L0X_SETUP_SIGMA_LIMIT          (60*65536)
#define VL53L0X_SETUP_MAX_TIME_FOR_RANGING     33000
#define VL53L0X_SETUP_PRE_RANGE_VCSEL_PERIOD   18
#define VL53L0X_SETUP_FINAL_RANGE_VCSEL_PERIOD 14
#define VL53L0X_CHIP_ADRESS_DEFAULT 0x29

struct vl53l0x_driver_config {
    char *xshut_gipos_controller;
    gpio_pin_t xshut_pin;
    char *bus_name;
    u8_t base_address;
};

struct vl53l0x_multi_driver_config {
    const struct vl53l0x_driver_config * configs;
    u8_t count;
    const struct vl53l0x_driver_config * self;
};

struct vl53l0x_data {
    struct device *i2c;
    VL53L0X_Dev_t vl53l0x;
    VL53L0X_RangingMeasurementData_t RangingMeasurementData;
};

static int vl53l0x_sample_fetch(struct device *dev, enum sensor_channel chan)
{
    struct vl53l0x_data *drv_data = dev->driver_data;
    VL53L0X_Error ret;

    __ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_DISTANCE || chan == SENSOR_CHAN_PROX);

    uint8_t NewDatReady = 0;
    ret = VL53L0X_GetMeasurementDataReady(&drv_data->vl53l0x, &NewDatReady);
    if (ret < 0) {
        LOG_ERR("Could not get ready data (error=%d)", ret);
        return -EIO;
    }

    if (NewDatReady != 0x01) {
        return -EBUSY;
    }

    ret = VL53L0X_GetRangingMeasurementData(&drv_data->vl53l0x, &drv_data->RangingMeasurementData);
    if (ret < 0) {
        LOG_ERR("Could not perform measurment (error=%d)", ret);
        return -EIO;
    }

    // Clear the interrupt
    VL53L0X_ClearInterruptMask(&drv_data->vl53l0x, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
    if (ret < 0) {
        LOG_ERR("Could not clear  (error=%d)", ret);
        return -EIO;
    }

    //VL53L0X_PollingDelay(&drv_data->vl53l0x);

    return 0;
}

static int vl53l0x_channel_get(struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
    struct vl53l0x_data *drv_data = (struct vl53l0x_data *) dev->driver_data;

    __ASSERT_NO_MSG(chan == SENSOR_CHAN_DISTANCE || chan == SENSOR_CHAN_PROX);

    if (chan == SENSOR_CHAN_PROX) {
        if (drv_data->RangingMeasurementData.RangeMilliMeter <= CONFIG_VL53L0X_PROXIMITY_THRESHOLD) {
            val->val1 = 1;
        } else {
            val->val1 = 0;
        }
        val->val2 = 0;
    } else {
        val->val1 = drv_data->RangingMeasurementData.RangeMilliMeter / 1000;
        val->val2 = (drv_data->RangingMeasurementData.RangeMilliMeter % 1000) * 1000;
    }

    if (drv_data->RangingMeasurementData.RangeStatus != 0) {
        return -EIO;
    }
    return 0;
}

static const struct sensor_driver_api vl53l0x_api_funcs = {
        .sample_fetch = vl53l0x_sample_fetch,
        .channel_get = vl53l0x_channel_get, };

static int vl53l0x_setup_continous(struct device *dev)
{
    struct vl53l0x_data *drv_data = dev->driver_data;
    int ret;
    u8_t VhvSettings;
    u8_t PhaseCal;
    u32_t refSpadCount;
    u8_t isApertureSpads;

    ret = VL53L0X_StaticInit(&drv_data->vl53l0x);
    if (ret) {
        LOG_ERR("VL53L0X_StaticInit failed");
        goto exit;
    }

    ret = VL53L0X_PerformRefCalibration(&drv_data->vl53l0x, &VhvSettings, &PhaseCal);
    if (ret) {
        LOG_ERR("VL53L0X_PerformRefCalibration failed");
        goto exit;
    }

    ret = VL53L0X_PerformRefSpadManagement(&drv_data->vl53l0x, (uint32_t *) &refSpadCount, &isApertureSpads);
    if (ret) {
        LOG_ERR("VL53L0X_PerformRefSpadManagement failed");
        goto exit;
    }

    ret = VL53L0X_SetDeviceMode(&drv_data->vl53l0x, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    if (ret) {
        LOG_ERR("VL53L0X_SetDeviceMode failed");
        goto exit;
    }

    ret = VL53L0X_SetLimitCheckEnable(&drv_data->vl53l0x, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    if (ret) {
        LOG_ERR("VL53L0X_SetLimitCheckEnable sigma failed");
        goto exit;
    }

    ret = VL53L0X_SetLimitCheckEnable(&drv_data->vl53l0x, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    if (ret) {
        LOG_ERR("VL53L0X_SetLimitCheckEnable signal rate failed");
        goto exit;
    }

    ret = VL53L0X_SetLimitCheckValue(&drv_data->vl53l0x, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
    VL53L0X_SETUP_SIGNAL_LIMIT);

    if (ret) {
        LOG_ERR("VL53L0X_SetLimitCheckValue signal rate failed");
        goto exit;
    }

    ret = VL53L0X_SetLimitCheckValue(&drv_data->vl53l0x, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
    VL53L0X_SETUP_SIGMA_LIMIT);
    if (ret) {
        LOG_ERR("VL53L0X_SetLimitCheckValue sigma failed");
        goto exit;
    }

    ret = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&drv_data->vl53l0x,
    VL53L0X_SETUP_MAX_TIME_FOR_RANGING);
    if (ret) {
        LOG_ERR("VL53L0X_SetMeasurementTimingBudgetMicroSeconds failed");
        goto exit;
    }

    ret = VL53L0X_SetVcselPulsePeriod(&drv_data->vl53l0x, VL53L0X_VCSEL_PERIOD_PRE_RANGE,
    VL53L0X_SETUP_PRE_RANGE_VCSEL_PERIOD);
    if (ret) {
        LOG_ERR("VL53L0X_SetVcselPulsePeriod pre range failed");
        goto exit;
    }

    ret = VL53L0X_SetVcselPulsePeriod(&drv_data->vl53l0x, VL53L0X_VCSEL_PERIOD_FINAL_RANGE,
    VL53L0X_SETUP_FINAL_RANGE_VCSEL_PERIOD);
    if (ret) {
        LOG_ERR("VL53L0X_SetVcselPulsePeriod final range failed");
        goto exit;
    }

    ret = VL53L0X_StartMeasurement(&drv_data->vl53l0x);
    if (ret) {
        LOG_ERR("VL53L0X_StartMeasurement failed");
        goto exit;
    }

    exit: return ret;
}

static VL53L0X_Error setupSensorAddress(struct vl53l0x_data *drv_data, const struct vl53l0x_driver_config *config)
{
    struct device *gpio = device_get_binding(config->xshut_gipos_controller);
    gpio_pin_set(gpio,config->xshut_pin, 1); // Enable Sensor
    LOG_DBG("Sensor %x enabled on port: %s pin: %x", config->base_address,config->xshut_gipos_controller,config->xshut_pin);
    k_sleep(K_MSEC(100));

    VL53L0X_Error ret;
    drv_data->i2c = device_get_binding(config->bus_name);
    if (drv_data->i2c == NULL) {
        LOG_ERR("Could not get pointer to %s device.", config->bus_name);
        return -EINVAL;
    }
    drv_data->vl53l0x.i2c = drv_data->i2c;
    drv_data->vl53l0x.I2cDevAddr = VL53L0X_CHIP_ADRESS_DEFAULT;

    uint8_t DeviceAddress = config->base_address & 0x7F;

    ret = VL53L0X_WrByte(&drv_data->vl53l0x, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, DeviceAddress);
    if (ret) {
        LOG_ERR("Could not write byte %x", DeviceAddress); // process further because address could already be set
    }

    LOG_DBG("Set new device address %x", DeviceAddress);
    drv_data->vl53l0x.I2cDevAddr = DeviceAddress;

    return 0;
}


static VL53L0X_Error init_pin(const struct vl53l0x_driver_config *config)
{
    struct device *gpio = device_get_binding(config->xshut_gipos_controller);
    if (gpio == NULL) {
        LOG_ERR("Could not get pointer to %s device.", config->xshut_gipos_controller);
        return -EINVAL;
    }

    if (gpio_pin_configure(gpio, config->xshut_pin, GPIO_OUTPUT | GPIO_PULL_UP) < 0) {
        LOG_ERR("Could not configure GPIO %s %d).", config->xshut_gipos_controller, config->xshut_pin);
        return -EINVAL;
    }

    gpio_pin_set(gpio,config->xshut_pin, 0);
    return 0;
}

static VL53L0X_Error setupSensors(const struct vl53l0x_multi_driver_config *config)
{
    static bool finished = false; //only initialize once the pins
    if (finished) {
        return 0;
    }
    // configure pins
    for (int i = 0; i < config->count; i++) {
        if(init_pin(&config->configs[i])){
            return -EINVAL;
        }
    }
    finished = true;
    LOG_DBG("reset all pins done");
    return 0;
}

static int vl53l0x_init(struct device *dev)
{
    const struct vl53l0x_multi_driver_config *config_all = dev->config->config_info;
    const struct vl53l0x_driver_config *config = config_all->self;

    VL53L0X_Error ret;

    ret = setupSensors(config_all);
    if (ret < 0) {
        LOG_ERR("Could setup sensors");
        return -ENODEV;
    }
    LOG_DBG("setupSensors done");

    struct vl53l0x_data *drv_data = dev->driver_data;

    u16_t vl53l0x_id = 0U;
    VL53L0X_DeviceInfo_t vl53l0x_dev_info;

    LOG_DBG("enter in %s", __func__);
    LOG_DBG("Sensor %x", config->base_address);


    ret = setupSensorAddress(drv_data, config);
    if (ret < 0) {
        LOG_ERR("Could setup sensors");
        return -ENODEV;
    }


    /* Get info from sensor */
    (void) memset(&vl53l0x_dev_info, 0, sizeof(VL53L0X_DeviceInfo_t));

    ret = VL53L0X_GetDeviceInfo(&drv_data->vl53l0x, &vl53l0x_dev_info);
    if (ret < 0) {
        LOG_ERR("Could not get info from device.");
        return -ENODEV;
    }

    LOG_DBG("VL53L0X_GetDeviceInfo = %d", ret);
    LOG_DBG("   Device Name : %s", vl53l0x_dev_info.Name);
    LOG_DBG("   Device Type : %s", vl53l0x_dev_info.Type);
    LOG_DBG("   Device ID : %s", vl53l0x_dev_info.ProductId);
    LOG_DBG("   ProductRevisionMajor : %d", vl53l0x_dev_info.ProductRevisionMajor);
    LOG_DBG("   ProductRevisionMinor : %d", vl53l0x_dev_info.ProductRevisionMinor);

    ret = VL53L0X_RdWord(&drv_data->vl53l0x,
    VL53L0X_REG_WHO_AM_I, (uint16_t *) &vl53l0x_id);
    if ((ret < 0) || (vl53l0x_id != VL53L0X_CHIP_ID)) {
        LOG_ERR("Issue on device identification");
        return -ENOTSUP;
    }

    /* sensor init */
    ret = VL53L0X_DataInit(&drv_data->vl53l0x);
    if (ret < 0) {
        LOG_ERR("VL53L0X_DataInit return error (%d)", ret);
        return -ENOTSUP;
    }

    ret = vl53l0x_setup_continous(dev);
    if (ret < 0) {
        return -ENOTSUP;
    }

    return 0;
}


#define NUMBER_OF_SENSORS 2

#define CREATE_DATA(DEV_ID) static struct vl53l0x_data vl53l0x_##DEV_ID##_data;

#define CREATE_CONFIG(DEV_ID) \
    {\
        .bus_name = DT_INST_##DEV_ID##_ST_VL53L0X_BUS_NAME, \
        .base_address = DT_INST_##DEV_ID##_ST_VL53L0X_BASE_ADDRESS, \
        .xshut_gipos_controller = DT_INST_##DEV_ID##_ST_VL53L0X_XSHUT_GPIOS_CONTROLLER, \
        .xshut_pin = DT_INST_##DEV_ID##_ST_VL53L0X_XSHUT_GPIOS_PIN \
    }

#define CREATE_DEVICE(DEV_ID) \
    DEVICE_AND_API_INIT(vl53l0x_##DEV_ID,DT_INST_##DEV_ID##_ST_VL53L0X_LABEL, vl53l0x_init, \
            &vl53l0x_##DEV_ID##_data,&vl53l0x_config_##DEV_ID, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,\
            &vl53l0x_api_funcs);

#define CREATE_MULTI_config(DEV_ID) \
static struct vl53l0x_multi_driver_config vl53l0x_config_##DEV_ID = { \
        .count = NUMBER_OF_SENSORS, \
        .configs = &configs[0], \
        .self = &configs[DEV_ID] \
};


CREATE_DATA(0)
CREATE_DATA(1)

struct vl53l0x_driver_config configs[] = { CREATE_CONFIG(0), CREATE_CONFIG(1) };

CREATE_MULTI_config(0)
CREATE_MULTI_config(1)


CREATE_DEVICE(0)
CREATE_DEVICE(1)

