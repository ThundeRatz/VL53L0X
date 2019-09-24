/**
 * @file vl53l0x.c
 *
 * @brief User functions to deal with ST's VL53L0X API
 *
 * @date 08/2019
 */

#include "vl53l0x.h"
#include "mcu.h"

#include "gpio.h"
#include "i2c.h"

/*****************************************
 * Private Constants
 *****************************************/

#define FILTER_VALUE 0.8

/*****************************************
 * Public Functions Bodies Definitions
 *****************************************/

VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t* p_device, VL53L0X_DeviceInfo_t device_info,
                           vl53l0x_calibration_data_t calibration) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    Status = VL53L0X_DataInit(p_device); // Data initialization

    if (Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_GetDeviceInfo(p_device, &device_info);

        if (Status == VL53L0X_ERROR_NONE)
        {
            if ((device_info.ProductRevisionMinor != 1) && (device_info.ProductRevisionMinor != 1))
            {
                Status = VL53L0X_ERROR_NOT_SUPPORTED;
            }
        }
    }

    if (Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_StaticInit(p_device); // Device Initialization
    }

    if (p_device->calibrated)
    {
        if (Status == VL53L0X_ERROR_NONE)
        {
            Status = VL53L0X_SetReferenceSpads(p_device,
                                               calibration.refSpadCount, calibration.isApertureSpads);
        }

        if (Status == VL53L0X_ERROR_NONE)
        {
            Status = VL53L0X_SetRefCalibration(p_device,
                                               calibration.VhvSettings, calibration.PhaseCal);
        }
    }
    else
    {
        if (Status == VL53L0X_ERROR_NONE)
        {
            Status =
                VL53L0X_PerformRefSpadManagement(p_device, &(calibration.refSpadCount),
                                                 &(calibration.isApertureSpads)); // Device Initialization
        }

        if (Status == VL53L0X_ERROR_NONE)
        {
            Status = VL53L0X_PerformRefCalibration(p_device, &(calibration.VhvSettings), &(calibration.PhaseCal)); // Device Initialization
        }
    }


    if (Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_SetDeviceMode(p_device, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in single ranging mode
    }

    // Enable Sigma Signal and Threshold check
    if (Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_SetLimitCheckEnable(p_device,
                                             VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    }

    if (Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_SetLimitCheckEnable(p_device,
                                             VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    }

    /*
 *  if (Status == VL53L0X_ERROR_NONE) {
 *      Status = VL53L0X_SetLimitCheckEnable(p_device,
 *              VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1);
 *  }
 * //*/

    // Set Values
    if (Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_SetLimitCheckValue(p_device,
                                            VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                                            (FixPoint1616_t)(18 * 65536)); // Default: 18mm
    }

    if (Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_SetLimitCheckValue(p_device,
                                            VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                            (FixPoint1616_t)(0.25 * 65536)); // Default: 0.25 Mcps
    }

    /*
 *  if (Status == VL53L0X_ERROR_NONE) {
 *      Status = VL53L0X_SetLimitCheckValue(p_device,
 *              VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
 *              (FixPoint1616_t)(1.5*0.023*65536));
 *  }
 * //*/

    if (Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(p_device,
                                                                50000); // Default: 33000us
    }

    // *
    if (Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_SetVcselPulsePeriod(p_device,
                                             VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18); // tem que ser entre 12 e 18
    }

    if (Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_SetVcselPulsePeriod(p_device,
                                             VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14); // tem q ser entre 8 e 14
    }

    // */

    if (Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_StartMeasurement(p_device);
    }

    return Status;
}

uint8_t vl53l0x_update_range(VL53L0X_Dev_t* p_device, VL53L0X_RangingMeasurementData_t* p_ranging_data,
                            uint16_t* range, uint16_t max_range) {
    uint8_t status = 0;
    status = VL53L0X_GetRangingMeasurementData(p_device, p_ranging_data);

    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }

    uint8_t range_status = p_ranging_data->RangeStatus; // TERMINAR A ANALISE DO STATUS

    if (range_status == 0) { //TUDO OKAY
        (*range) = (p_ranging_data->RangeMilliMeter) * FILTER_VALUE + (1 - FILTER_VALUE) * (*range);

    } else if (range_status == 1) { //SIGMA FAIL
        (*range) = (p_ranging_data->RangeMilliMeter) * 0.2 + (1 - 0.2) * (*range);
    } else if (range_status == 4) { //PHASE FAIL
        //Melhor não fazer nada?
    } else if (range_status == 5) { //HARDWARE FAIL
        (*range) = max_range;
        //Desabilitar o sensor?
    } else {
        /*
         * 1 - SIGMA FAIL
         * 2 - SIGNAL FAIL
         * 3 - MIN RANGE FAIL
         */
        (*range) = (p_ranging_data->RangeMilliMeter) * 0.4 + (1 - 0.4) * (*range);
    }


    if ((*range) > max_range) {
        (*range) = max_range;
    }

    if (status == 0) {
        status = range_status;
    }

    return status;
}

void vl53l0x_turn_off(VL53L0X_Dev_t* p_device) {
    HAL_GPIO_WritePin(p_device->xshut_port, p_device->xshut_pin, GPIO_PIN_RESET);
}

void vl53l0x_turn_on(VL53L0X_Dev_t* p_device) {
    HAL_GPIO_WritePin(p_device->xshut_port, p_device->xshut_pin, GPIO_PIN_SET);
}

VL53L0X_Error vl53l0x_wait_boot(VL53L0X_Dev_t* p_device) {
    VL53L0X_Error status = -3;
    uint16_t byte = 0x0000;
    uint16_t loopCounter = 0;

    vl53l0x_turn_on(p_device);

    while (loopCounter < 2000)
    {
        mcu_sleep(0); // SEM ESSE DELAY NÃO FUNCIONA
        status = VL53L0X_RdWord(p_device,
                                VL53L0X_REG_IDENTIFICATION_MODEL_ID, &byte);

        if (byte == 0xEEAA)
        {
            break;
        }

        loopCounter++;
    }

    return status;
}

// TODO: conferir API State quando tira o xshut
// PALDevDataGet(Dev, PalState)
