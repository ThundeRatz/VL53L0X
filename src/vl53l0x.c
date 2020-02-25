/**
 * @file vl53l0x.c
 *
 * @brief User functions to deal with ST's VL53L0X API
 *
 * @author Lucas Schneider <lucas.schneider@thunderatz.org>
 *
 * @date 10/2019
 */

#include "vl53l0x.h"
#include "gpio.h"

/*****************************************
 * Private Macro
 *****************************************/

/**
 * @brief Returns minimum value between x and y.
 */
#define min(x, y) ((x) < (y) ? (x) : (y))

/*****************************************
 * Private Constants
 *****************************************/

#define SIGMA_LIMIT_VALUE_MM 60
#define SIGNAL_RATE_LIMIT_VALUE_MCPS 0.1
#define MEASUREMENT_TIME_BUDGET_US 40000
#define PRE_RANGE_PULSE_PERIOD 18     // Must be between 12 and 18
#define FINAL_RANGE_PULSE_PERIOD 14   // Must be between 8 and 14

/**
 * @brief Filter values, depending on range status
 */
#define VALID_RANGE_FILTER 0.8
#define SIGMA_FAIL_FILTER 0.2
#define DAFAULT_FILTER 0.4

/*****************************************
 * Public Functions Bodies Definitions
 *****************************************/

VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t* p_device, VL53L0X_DeviceInfo_t device_info,
                           vl53l0x_calibration_data_t calibration) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    // Data initialization
    Status = VL53L0X_DataInit(p_device);

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_GetDeviceInfo(p_device, &device_info);

        if (Status == VL53L0X_ERROR_NONE) {
            if ((device_info.ProductRevisionMinor != 1) && (device_info.ProductRevisionMinor != 1)) {
                Status = VL53L0X_ERROR_NOT_SUPPORTED;
            }
        }
    }

    if (Status == VL53L0X_ERROR_NONE) {
        // Device Initialization
        Status = VL53L0X_StaticInit(p_device);
    }

    if (p_device->calibrated) {
        // Load de calibration data
        if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_SetReferenceSpads(p_device,
                                               calibration.refSpadCount, calibration.isApertureSpads);
        }

        if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_SetRefCalibration(p_device,
                                               calibration.VhvSettings, calibration.PhaseCal);
        }
    } else {
        // Device calibration
        if (Status == VL53L0X_ERROR_NONE) {
            Status =
                VL53L0X_PerformRefSpadManagement(p_device, &(calibration.refSpadCount),
                                                 &(calibration.isApertureSpads));
        }

        if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_PerformRefCalibration(p_device, &(calibration.VhvSettings), &(calibration.PhaseCal));
        }
    }

    if (Status == VL53L0X_ERROR_NONE) {
        // Device default is single ranging mode
        Status = VL53L0X_SetDeviceMode(p_device, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    }

    // Enable Sigma Signal and Threshold check
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(p_device,
                                             VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(p_device,
                                             VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    }

    // Set Values
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(p_device,
                                            VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                                            (FixPoint1616_t) (SIGMA_LIMIT_VALUE_MM * (1 << 16)));
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(p_device,
                                            VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                            (FixPoint1616_t) (SIGNAL_RATE_LIMIT_VALUE_MCPS * (1 << 16)));
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(p_device, MEASUREMENT_TIME_BUDGET_US);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetVcselPulsePeriod(p_device,
                                             VL53L0X_VCSEL_PERIOD_PRE_RANGE, PRE_RANGE_PULSE_PERIOD);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetVcselPulsePeriod(p_device,
                                             VL53L0X_VCSEL_PERIOD_FINAL_RANGE, FINAL_RANGE_PULSE_PERIOD);
    }

    // Start reading
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_StartMeasurement(p_device);
    }

    return Status;
}

void vl53l0x_turn_off(VL53L0X_Dev_t* p_device) {
    HAL_GPIO_WritePin(p_device->xshut_port, p_device->xshut_pin, GPIO_PIN_RESET);
}

void vl53l0x_turn_on(VL53L0X_Dev_t* p_device) {
    HAL_GPIO_WritePin(p_device->xshut_port, p_device->xshut_pin, GPIO_PIN_SET);
}

VL53L0X_Error vl53l0x_wait_boot(VL53L0X_Dev_t* p_device) {
    VL53L0X_Error status = VL53L0X_ERROR_UNDEFINED;
    uint16_t byte = 0x0000;
    uint16_t loopCounter = 0;

    vl53l0x_turn_on(p_device);

    while (loopCounter < 400) {
        // It doesn't work without a delay, even using 0
        VL53L0X_Delay(0);
        status = VL53L0X_RdWord(p_device,
                                VL53L0X_REG_IDENTIFICATION_MODEL_ID, &byte);

        if (byte == 0xEEAA) {
            break;
        }

        loopCounter++;
    }

    return status;
}

uint8_t vl53l0x_update_range(VL53L0X_Dev_t* p_device, VL53L0X_RangingMeasurementData_t* p_ranging_data, uint16_t* range,
                             uint16_t max_range) {
    uint8_t status = 0;
    uint16_t aux_range = *range;
    status = VL53L0X_GetRangingMeasurementData(p_device, p_ranging_data);

    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }

    uint8_t range_status = p_ranging_data->RangeStatus;

    if (range_status == 0) {  // VALID RANGE
        aux_range = (p_ranging_data->RangeMilliMeter) * VALID_RANGE_FILTER + (1 - VALID_RANGE_FILTER) * aux_range;
    } else if (range_status == 1) {  // SIGMA FAIL
        aux_range = (p_ranging_data->RangeMilliMeter) * SIGMA_FAIL_FILTER + (1 - SIGMA_FAIL_FILTER) * aux_range;
    } else if (range_status == 4) {  // PHASE FAIL
        /* In this case, aux_range will not be updated, because the reading is mostly random when this erro occurs */
    } else if (range_status == 5) {  // HARDWARE FAIL
        aux_range = max_range;
    } else {
        // 2 - SIGNAL FAIL
        // 3 - MIN RANGE FAIL
        aux_range = (p_ranging_data->RangeMilliMeter) * DAFAULT_FILTER + (1 - DAFAULT_FILTER) * aux_range;
    }

    *range = min(aux_range, max_range);

    if (status == 0) {
        status = range_status;
    }

    return status;
}
