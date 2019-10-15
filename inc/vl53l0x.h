/**
 * @file vl53l0x.h
 *
 * @brief User functions to deal with ST's VL53L0X API
 *
 * @author Lucas Schneider <lucas.schneider@thunderatz.org>
 *
 * @date 10/2019
 */

#ifndef VL53L0X_H
#define VL53L0X_H

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

/*****************************************
 * Public Constants
 *****************************************/

#define VL53L0X_DEFAULT_ADDRESS 0x52

/*****************************************
 * Public Types
 *****************************************/

/**
 * @brief Struct to storage the calibration data for one sensor, so the values can be reloaded after reboot
 */
typedef struct _calibration_data {
    uint32_t refSpadCount;
    uint8_t  isApertureSpads;
    uint8_t  VhvSettings;
    uint8_t  PhaseCal;
} vl53l0x_calibration_data_t;

/*****************************************
 * Public Functions Prototypes
 *****************************************/

/**
 * @brief Initializes VL53L0X device
 *
 * @note Before initialization, check if the device can communicate with @ref vl53l0x_wait_boot
 *
 * @param p_device      Device handler
 * @param device_info   Device informations
 * @param calibration   Device calibration data
 *
 * @return Error code
 * @retval VL53L0X_ERROR_NONE   Success
 * @retval "Other error code"   More details in @ref VL53L0X_Error
 */
VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t* p_device, VL53L0X_DeviceInfo_t device_info,
                           vl53l0x_calibration_data_t calibration);

/**
 * @brief Turns off device with the XSHUT pin
 *
 * @param p_device      Device handler
 */
void vl53l0x_turn_off(VL53L0X_Dev_t* p_device);

/**
 * @brief Turns on device with the XSHUT pin
 *
 * @param p_device      Device handler
 */
void vl53l0x_turn_on(VL53L0X_Dev_t* p_device);

/**
 * @brief Turns on device and check I2C communication
 *
 * @param p_device      Device handler
 *
 * @return Error code
 * @retval VL53L0X_ERROR_NONE   Success
 * @retval "Other error code"   More details in @ref VL53L0X_Error
 */
VL53L0X_Error vl53l0x_wait_boot(VL53L0X_Dev_t* p_device);

/**
 * @brief Update sensor range
 *
 * @param p_device          Device handler
 * @param p_ranging_data    Device measurement data
 * @param range             Adress to store the measurement in mm
 * @param max_range         Max range the sensor can read in mm
 *
 * @return Error code
 * @retval 0                        Success
 * @retval "Number between 1 and 5" Range status from sensor reading @see API_User_Manual
 * @retval "Other error code"       More details in @ref VL53L0X_Error
 */
uint8_t vl53l0x_update_range(VL53L0X_Dev_t* p_device, VL53L0X_RangingMeasurementData_t* p_ranging_data, uint16_t* range,
                             uint16_t max_range);

uint8_t check_API_status();  // NOT IMPLEMENTED YET

uint8_t vl53l0x_reinit();  // NOT IMPLEMENTED YET

#endif // VL53L0X_H
