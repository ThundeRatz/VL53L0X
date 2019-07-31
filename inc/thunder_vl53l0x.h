#ifndef THUNDER_VL53L0X_H
#define THUNDER_VL53L0X_H

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

typedef enum _distance_sensor_position {
    DS_SIDE_RIGHT = 0,
    DS_FRONT_RIGHT = 1,
    DS_FRONT_CENTER = 2,
    DS_FRONT_LEFT = 3,
    DS_SIDE_LEFT = 4,

    DS_MAX_POSITION = 5
} distance_sensor_position_t;

typedef struct _calibration_data {
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
} VL53L0X_CalibrationData_t;

uint8_t vl53l0x_init();

VL53L0X_Error vl53l0x_single_init(VL53L0X_DEV pMyDevice,
    VL53L0X_DeviceInfo_t DeviceInfo, VL53L0X_CalibrationData_t calibration);

uint8_t rangingTest();

uint8_t check_API_status(); //TODO

uint8_t vl53l0x_reinit(); //TODO


// FUNCÕES DA BIBLIOTECA ANTIGA PARA IMPLEMENTAR E DEIXAR O CÓDIGO COMPATÍVEL
uint8_t vl53l0x_update(void); // TODO: consertar

uint8_t just_update();

uint16_t vl53l0x_get(distance_sensor_position_t sensor);


#endif
