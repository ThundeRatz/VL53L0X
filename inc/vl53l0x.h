/**
 * @file vl53l0x.h
 *
 * @brief User functions to deal with ST's VL53L0X API
 *
 * @date 08/2019
 */

#ifndef VL53L0X_H
#define VL53L0X_H

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

/*****************************************
 * Public Constants
 *****************************************/

#define VL53L0X_DEFAULT_COMM_SPEED 100 // SE TROCAR, TEM QUE TROCAR NO CUBE
#define VL53L0X_DEFAULT_ADDRESS 0x52

/*****************************************
 * Public Types
 *****************************************/

typedef struct _calibration_data  {
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
} vl53l0x_calibration_data_t;

/*****************************************
 * Public Functions Prototypes
 *****************************************/

//TODO: selecão entre o modos de operação do dispoitivo
VL53L0X_Error vl53l0x_init(VL53L0X_DEV pMyDevice, VL53L0X_DeviceInfo_t DeviceInfo,
                        vl53l0x_calibration_data_t calibration);


//TODO: Terminar a análise do RangeStatus fornecido pela API
uint8_t vl53l0x_update_range(VL53L0X_Dev_t* pMyDevice, VL53L0X_RangingMeasurementData_t *pRangingData,
                        uint16_t* range, uint16_t max_range);


void vl53l0x_turn_off(VL53L0X_DEV Dev);

void vl53l0x_turn_on(VL53L0X_DEV Dev);

//TODO: Implementar de melhor, principalmente parte de esperar ligar
VL53L0X_Error vl53l0x_wait_boot(VL53L0X_DEV Dev);

uint8_t check_API_status(); //TODO

uint8_t vl53l0x_reinit(); //TODO

#endif
