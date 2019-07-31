#include "thunder_vl53l0x.h"
#include "i2c.h"
#include "mcu.h"

#include <stdio.h>
#include "gpio.h"

#define VL53L0X_DEFAULT_ADDRESS 0x52   // NÃO MEXER
#define VL53L0X_DEFAULT_COMM_SPEED 100 // SE TROCAR, TEM QUE TROCAR NO CUBE

#define DS_AMOUNT 5

#define INIT_RESET_SLEEP_TIME_MS 10 // TODO: pode diminuir isso?

#define MAX_RANGE 600
#define FILTER_VALUE 0.4

static VL53L0X_Dev_t sensors[] = {
    { // 0
        .I2cDevAddr = VL53L0X_DEFAULT_ADDRESS,
        .comms_type = 1, // I2C
        .comms_speed_khz = VL53L0X_DEFAULT_COMM_SPEED,
        .present = 0,
        .calibrated = 0,
        .hi2c = &hi2c1,
        .xshut_port = GPIOA,
        .xshut_pin = GPIO_PIN_7
    },
    { // 1
        .I2cDevAddr = VL53L0X_DEFAULT_ADDRESS,
        .comms_type = 1, // I2C
        .comms_speed_khz = VL53L0X_DEFAULT_COMM_SPEED,
        .present = 0,
        .calibrated = 0,
        .hi2c = &hi2c1,
        .xshut_port = GPIOB,
        .xshut_pin = GPIO_PIN_0
    },
    { // 2
        .I2cDevAddr = VL53L0X_DEFAULT_ADDRESS,
        .comms_type = 1, // I2C
        .comms_speed_khz = VL53L0X_DEFAULT_COMM_SPEED,
        .present = 0,
        .calibrated = 0,
        .hi2c = &hi2c1,
        .xshut_port = GPIOC,
        .xshut_pin = GPIO_PIN_7
    },
    { // 3
        .I2cDevAddr = VL53L0X_DEFAULT_ADDRESS,
        .comms_type = 1, // I2C
        .comms_speed_khz = VL53L0X_DEFAULT_COMM_SPEED,
        .present = 0,
        .calibrated = 0,
        .hi2c = &hi2c1,
        .xshut_port = GPIOA,
        .xshut_pin = GPIO_PIN_9
    },
    { // 4
        .I2cDevAddr = VL53L0X_DEFAULT_ADDRESS,
        .comms_type = 1, // I2C
        .comms_speed_khz = VL53L0X_DEFAULT_COMM_SPEED,
        .present = 0,
        .calibrated = 0,
        .hi2c = &hi2c1,
        .xshut_port = GPIOA,
        .xshut_pin = GPIO_PIN_8
    }
};

static VL53L0X_DeviceInfo_t sensorsInfo[DS_MAX_POSITION];
static VL53L0X_CalibrationData_t sensorsCalibration[DS_MAX_POSITION];
static VL53L0X_RangingMeasurementData_t sensorsMeasurement[DS_MAX_POSITION];

static uint16_t actualRange[] = {0, 0, 0, 0, 0};
static const uint8_t i2c_addresses[] = {0x30, 0x34, 0x38, 0x3C, 0x40};

uint8_t vl53l0x_init() {
    VL53L0X_Error Global_Status = VL53L0X_ERROR_NONE;

    MX_I2C1_Init(); // TODO: fazer checagem de erro Error_Handler(); (cube_main.c)

    // desabilita todos, independente de quantos vai usar
    for (int i = 0; i < DS_MAX_POSITION; i++) {
        VL53L0X_TurnOff(&(sensors[i]));
    }

    mcu_sleep(INIT_RESET_SLEEP_TIME_MS);

    for (int i = 0; i < DS_AMOUNT; i++) {
        VL53L0X_Error Status = VL53L0X_ERROR_NONE;
        VL53L0X_DEV pDevice = &(sensors[i]);

        Status = VL53L0X_TurnOn_WaitBoot(pDevice);

        if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_SetDeviceAddress(pDevice, i2c_addresses[i]);
        }

        if (Status == VL53L0X_ERROR_NONE) {
            pDevice->I2cDevAddr = i2c_addresses[i];
            Status = vl53l0x_single_init(pDevice, sensorsInfo[i], sensorsCalibration[i]);
        }

        if (Status == VL53L0X_ERROR_NONE) {
            pDevice->present = 1;
            pDevice->calibrated = 1;
        }

        Global_Status |= Status;
    }

    if (Global_Status == VL53L0X_ERROR_NONE) {
        return 0;
    } else {
        return 1;
    }
}

VL53L0X_Error vl53l0x_single_init(VL53L0X_DEV pMyDevice, VL53L0X_DeviceInfo_t DeviceInfo,
                                  VL53L0X_CalibrationData_t calibration) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    Status = VL53L0X_DataInit(pMyDevice); // Data initialization

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_GetDeviceInfo(pMyDevice, &DeviceInfo);

        if (Status == VL53L0X_ERROR_NONE) {
            if ((DeviceInfo.ProductRevisionMinor != 1) && (DeviceInfo.ProductRevisionMinor != 1)) {
                Status = VL53L0X_ERROR_NOT_SUPPORTED;
            }
        }
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_StaticInit(pMyDevice); // Device Initialization
    }

    if (pMyDevice->calibrated) {
        if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_SetReferenceSpads(pMyDevice,
                                               calibration.refSpadCount, calibration.isApertureSpads);
        }

        if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_SetRefCalibration(pMyDevice,
                                               calibration.VhvSettings, calibration.PhaseCal);
        }
    } else {
        if (Status == VL53L0X_ERROR_NONE) {
            Status =
                VL53L0X_PerformRefSpadManagement(pMyDevice, &(calibration.refSpadCount),
                                                 &(calibration.isApertureSpads)); // Device Initialization
        }

        if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_PerformRefCalibration(pMyDevice, &(calibration.VhvSettings), &(calibration.PhaseCal)); // Device Initialization
        }
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in single ranging mode
    }

    // Enable Sigma Signal and Threshold check
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
                                             VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
                                             VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    }

/*
 *  if (Status == VL53L0X_ERROR_NONE) {
 *      Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
 *              VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1);
 *  }
 * //*/

    // Set Values
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(pMyDevice,
                                            VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                                            (FixPoint1616_t) (18 * 65536)); // Default: 18mm
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(pMyDevice,
                                            VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                            (FixPoint1616_t) (0.25 * 65536)); // Default: 0.25 Mcps
    }

/*
 *  if (Status == VL53L0X_ERROR_NONE) {
 *      Status = VL53L0X_SetLimitCheckValue(pMyDevice,
 *              VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
 *              (FixPoint1616_t)(1.5*0.023*65536));
 *  }
 * //*/

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pMyDevice,
                                                                50000); // Default: 33000us
    }

// *
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetVcselPulsePeriod(pMyDevice,
                                             VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18); // tem que ser entre 12 e 18
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetVcselPulsePeriod(pMyDevice,
                                             VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14); // tem q ser entre 8 e 14
    }

// */

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_StartMeasurement(pMyDevice);
    }

    return Status;
}

uint8_t rangingTest(uint16_t distance[DS_AMOUNT]) {
    // only for testing purposes
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    for (int i = 0; i < DS_AMOUNT; i++) {
        VL53L0X_Dev_t* pMyDevice = &sensors[i];
        VL53L0X_RangingMeasurementData_t* pRangingMeasurementData = &sensorsMeasurement[i];

        Status = VL53L0X_GetRangingMeasurementData(pMyDevice, pRangingMeasurementData);

        if (Status == VL53L0X_ERROR_NONE) {
            distance[i] = pRangingMeasurementData->RangeMilliMeter;
        }
    }

    if (Status == VL53L0X_ERROR_NONE) {
        return 0;
    } else {
        return 1;
    }
}

/*
 * Essa função esperaria a medida de cada sensor ficar pronta para pegar o resultado mais atual possivel
 * Não ta funcionando direito
 * Necessário fazer asim, ou melhor só atualizar com o ultimo valor lido??? (ver just_update())
 */
uint8_t vl53l0x_update() {
    VL53L0X_Error Global_Status = VL53L0X_ERROR_NONE;
    VL53L0X_Error Single_Status[DS_AMOUNT];
    uint8_t newDataReady[DS_AMOUNT];
    uint16_t loopCount = 0;
    uint8_t sensorsNotRead = DS_AMOUNT;

    for (int i = 0; i < DS_AMOUNT; i++) {
        newDataReady[i] = 0;
        Single_Status[i] = VL53L0X_ERROR_NONE;
    }

    while (sensorsNotRead > 0) {
        if (loopCount >= VL53L0X_DEFAULT_MAX_LOOP) {
            for (int i = 0; i < DS_AMOUNT; i++) {
                if (!newDataReady[i]) {
                    (sensors[i]).present = 0;
                }
            }

            Global_Status = VL53L0X_ERROR_TIME_OUT;
            break;
        }

        for (int i = 0; i < DS_AMOUNT; i++) {
            if (!newDataReady[i]) {
                VL53L0X_Dev_t* pMyDevice = &sensors[i];
                Single_Status[i] = VL53L0X_GetMeasurementDataReady(pMyDevice, &newDataReady[i]);

                if (Single_Status[i] == VL53L0X_ERROR_NONE) {
                    if (newDataReady[i] == 0x01) {
                        VL53L0X_RangingMeasurementData_t* pRangingData = &(sensorsMeasurement[i]);
                        Single_Status[i] = VL53L0X_GetRangingMeasurementData(pMyDevice, pRangingData);

                        if (pRangingData->RangeStatus != 0) {
                            // newDataReady[i] = 0;
                            pRangingData->RangeMilliMeter = 0;
                        }

                        VL53L0X_ClearInterruptMask(pMyDevice, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
                        sensorsNotRead--;
                    }
                } else {
                    newDataReady[i] = 1; // para não tentar pegar a medida novamente
                    sensorsNotRead--;
                    Global_Status = 1; // passa o erro
                    pMyDevice->present = 0;
                }
            }
        }

        loopCount++;
    }

    return Global_Status;
}

/*
 * Essa apenas pega o ultimo valor lido, aplica o filtro
 * TODO: Terminar a análise do RangeStatus fornecido pela API
 */
uint8_t just_update() {
    uint8_t Status = 0;

    for (int i = 0; i < DS_AMOUNT; i++) {
        VL53L0X_Dev_t* pMyDevice = &sensors[i];
        VL53L0X_RangingMeasurementData_t* pRangingData = &(sensorsMeasurement[i]);
        VL53L0X_GetRangingMeasurementData(pMyDevice, pRangingData);

        switch (pRangingData->RangeStatus) {
            case 0: // TUDO OKAY
                actualRange[i] = (pRangingData->RangeMilliMeter) * FILTER_VALUE + (1 - FILTER_VALUE) * actualRange[i];
                break;

            case 4: // PHASE FAIL
                // Melhor não fazer nada e apenas usar a leitura anterior do sensor
                // Warning???
                break;

            case 5: // HARDWARE FAIL
                actualRange[i] = MAX_RANGE;

                // DESABILITAR O SENSOR???
                break;

            /*
             * 1 - SIGMA FAIL
             * 2 - SIGNAL FAIL
             * 3 - MIN RANGE FAIL
             */
            default:
                actualRange[i] = (pRangingData->RangeMilliMeter) * FILTER_VALUE + (1 - FILTER_VALUE) * actualRange[i];

                // FAZER ALGUM TIPO DE WARNING?
        }

        if (actualRange[i] > MAX_RANGE) {
            actualRange[i] = MAX_RANGE;
        }

// Status |= pRangingData->RangeStatus; TERMINAR A ANALISE DO STATUS
    }

    return Status;
}

uint16_t vl53l0x_get(distance_sensor_position_t sensor) {
    if ((sensors[(int) sensor]).present) {
        return actualRange[(int) sensor];
    } else {
        return -1;
    }
}

uint8_t vl53l0x_reinit() {
    return 1;
}

// TODO: conferir API State quando tira o xshut
// PALDevDataGet(Dev, PalState)
