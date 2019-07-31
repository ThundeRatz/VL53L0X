/*
 * COPYRIGHT (C) STMicroelectronics 2015. All rights reserved.
 *
 * This software is the confidential and proprietary information of
 * STMicroelectronics ("Confidential Information").  You shall not
 * disclose such Confidential Information and shall use it only in
 * accordance with the terms of the license agreement you entered into
 * with STMicroelectronics
 *
 * Programming Golden Rule: Keep it Simple!
 *
 */

/**
 * @file   VL53L0X_i2c_stm32g0xx.c
 * @brief  Code function defintions for ST Nucleo xxxxxx?
 *
 */

#include <string.h>

#include "vl53l0x_i2c_platform.h"
#include "vl53l0x_def.h"

#if defined STM32G0
#include "stm32g0xx_hal.h"
#elif defined STM32F3
#include "stm32f3xx_hal.h"
#endif

#include "i2c.h"

/*****************************************************************************
 * Private global constants.
 *****************************************************************************/

/**
 * @brief I2C timeout.
 */
#define I2C_TIMEOUT_FIX_MS (10)
#define I2C_TIMEOUT_PER_BYTE_MS (1)

#define I2C_TIMEOUT_MS(count) (I2C_TIMEOUT_FIX_MS + (count) * I2C_TIMEOUT_PER_BYTE_MS)

#define I2C_HANDLER hi2c1

#define MAX_DEVICES 5
#define STATUS_OK 0x00
#define STATUS_FAIL 0x01

/*****************************************************************************
 * Private global variables.
 *****************************************************************************/

/**
 * @brief I2C global buffer.
 */
static uint8_t mp_i2c_buffer[COMMS_BUFFER_SIZE];

/*****************************************************************************
 * Bodies of public functions.
 *****************************************************************************/

int VL53L0X_i2c_init(char* comPortStr, unsigned int baudRate) {
    // mja
    unsigned int status = STATUS_FAIL;

    return status;
}

int32_t VL53L0X_comms_close(void) {
    unsigned int status = STATUS_FAIL;

    return status;
}

int32_t VL53L0X_write_multi(uint8_t address, uint8_t index, uint8_t* pdata, int32_t count) {
    int32_t status = STATUS_OK;

    mp_i2c_buffer[0] = index;
    memcpy(&mp_i2c_buffer[1], pdata, count);

    if (HAL_OK != HAL_I2C_Master_Transmit(&I2C_HANDLER, address, mp_i2c_buffer, count + 1, I2C_TIMEOUT_MS(count))) {
        status = STATUS_FAIL;
    }

    return status;
}

int32_t VL53L0X_read_multi(uint8_t address, uint8_t index, uint8_t* pdata, int32_t count) {
    int32_t status = STATUS_OK;

    if (HAL_OK != HAL_I2C_Mem_Read(&I2C_HANDLER, address, index, 0x00000001U, pdata, count, I2C_TIMEOUT_MS(count))) {
        status = STATUS_FAIL;
    }

    // jeito do kim
    // if (HAL_OK != HAL_I2C_Master_Transmit(&I2C_HANDLER, address, &index, 1, I2C_TIMEOUT_MS(1)))
    // return STATUS_FAIL;

    // if (HAL_OK != HAL_I2C_Master_Receive(&I2C_HANDLER, address | 1, pdata, count, I2C_TIMEOUT_MS(count)))
    // status = STATUS_FAIL;

    return status;
}

int32_t VL53L0X_write_byte(uint8_t address, uint8_t index, uint8_t data) {
    int32_t status = STATUS_OK;
    const int32_t cbyte_count = 1;
    status = VL53L0X_write_multi(address, index, &data, cbyte_count);
    return status;
}

int32_t VL53L0X_write_word(uint8_t address, uint8_t index, uint16_t data) {
    int32_t status = STATUS_OK;

    uint8_t buffer[BYTES_PER_WORD];

    // Split 16-bit word into MS and LS uint8_t
    buffer[0] = (uint8_t) (data >> 8);
    buffer[1] = (uint8_t) (data & 0x00FF);

    if (index % 2 == 1) {
        status = VL53L0X_write_multi(address, index, &buffer[0], 1);
        status = VL53L0X_write_multi(address, index + 1, &buffer[1], 1);

        // serial comms cannot handle word writes to non 2-byte aligned registers.
    } else {
        status = VL53L0X_write_multi(address, index, buffer, BYTES_PER_WORD);
    }

    return status;
}

int32_t VL53L0X_write_dword(uint8_t address, uint8_t index, uint32_t data) {
    int32_t status = STATUS_OK;
    uint8_t buffer[BYTES_PER_DWORD];

    // Split 32-bit word into MS ... LS bytes
    buffer[0] = (uint8_t) (data >> 24);
    buffer[1] = (uint8_t) ((data & 0x00FF0000) >> 16);
    buffer[2] = (uint8_t) ((data & 0x0000FF00) >> 8);
    buffer[3] = (uint8_t) (data & 0x000000FF);

    status = VL53L0X_write_multi(address, index, buffer, BYTES_PER_DWORD);

    return status;
}

int32_t VL53L0X_read_byte(uint8_t address, uint8_t index, uint8_t* pdata) {
    int32_t status = STATUS_OK;
    int32_t cbyte_count = 1;

    status = VL53L0X_read_multi(address, index, pdata, cbyte_count);

    return status;
}

int32_t VL53L0X_read_word(uint8_t address, uint8_t index, uint16_t* pdata) {
    int32_t status = STATUS_OK;
    uint8_t buffer[BYTES_PER_WORD];

    status = VL53L0X_read_multi(address, index, buffer, BYTES_PER_WORD);
    *pdata = ((uint16_t) buffer[0] << 8) + (uint16_t) buffer[1];

    return status;
}

int32_t VL53L0X_read_dword(uint8_t address, uint8_t index, uint32_t* pdata) {
    int32_t status = STATUS_OK;
    uint8_t buffer[BYTES_PER_DWORD];

    status = VL53L0X_read_multi(address, index, buffer, BYTES_PER_DWORD);
    *pdata = ((uint32_t) buffer[0] << 24) + ((uint32_t) buffer[1] << 16) + ((uint32_t) buffer[2] << 8) +
             (uint32_t) buffer[3];

    return status;
}

// 16 bit address functions

/****************************************
 *
 * int32_t VL53L0X_write_multi16(uint8_t address, uint16_t index, uint8_t *pdata, int32_t count)
 * {
 *  int32_t status = STATUS_OK;
 *  unsigned int retries = 3;
 *  DWORD dwWaitResult;
 *
 *
 *  dwWaitResult = WaitForSingleObject(ghMutex, INFINITE);
 *  if(dwWaitResult == WAIT_OBJECT_0)
 *  {
 *      do
 *      {
 *          status = SERIAL_COMMS_Write_UBOOT(address, 0, index, pdata, count);
 *          // note : the field dwIndexHi is ignored. dwIndexLo will
 *          // contain the entire index (bits 0..15).
 *          if(status != STATUS_OK)
 *          {
 *              SERIAL_COMMS_Get_Error_Text(debug_string);
 *          }
 *      } while ((status != 0) && (retries-- > 0));
 *      ReleaseMutex(ghMutex);
 *  }
 *
 *  // store the page from the high byte of the index
 *  cached_page = HIBYTE(index);
 *
 *  if(status != STATUS_OK)
 *  {
 *      SERIAL_COMMS_Get_Error_Text(debug_string);
 *  }
 *
 *
 *  return status;
 * }
 *
 * int32_t VL53L0X_read_multi16(uint8_t address, uint16_t index, uint8_t *pdata, int32_t count)
 * {
 *  int32_t status = STATUS_OK;
 *  unsigned int retries = 3;
 *  DWORD dwWaitResult;
 *
 *  dwWaitResult = WaitForSingleObject(ghMutex, INFINITE);
 *  if(dwWaitResult == WAIT_OBJECT_0)
 *  {
 *      do
 *      {
 *          status = SERIAL_COMMS_Read_UBOOT(address, 0, index, pdata, count);
 *          if(status != STATUS_OK)
 *          {
 *              SERIAL_COMMS_Get_Error_Text(debug_string);
 *          }
 *      } while ((status != 0) && (retries-- > 0));
 *      ReleaseMutex(ghMutex);
 *  }
 *
 *  // store the page from the high byte of the index
 *  cached_page = HIBYTE(index);
 *
 *  if(status != STATUS_OK)
 *  {
 *      SERIAL_COMMS_Get_Error_Text(debug_string);
 *  }
 *
 *  return status;
 * }
 *
 *
 *
 * int32_t VL53L0X_write_byte16(uint8_t address, uint16_t index, uint8_t data)
 * {
 *  int32_t status = STATUS_OK;
 *  const int32_t cbyte_count = 1;
 *
 *
 *  status = VL53L0X_write_multi16(address, index, &data, cbyte_count);
 *
 *  return status;
 *
 * }
 *
 *
 * int32_t VL53L0X_write_word16(uint8_t address, uint16_t index, uint16_t data)
 * {
 *  int32_t status = STATUS_OK;
 *
 *  uint8_t  buffer[BYTES_PER_WORD];
 *
 *  // Split 16-bit word into MS and LS uint8_t
 *  buffer[0] = (uint8_t)(data >> 8);
 *  buffer[1] = (uint8_t)(data &  0x00FF);
 *
 *  if(index%2 == 1)
 *  {
 *      status = VL53L0X_write_multi16(address, index, &buffer[0], 1);
 *      status = VL53L0X_write_multi16(address, index + 1, &buffer[1], 1);
 *      // serial comms cannot handle word writes to non 2-byte aligned registers.
 *  }
 *  else
 *  {
 *      status = VL53L0X_write_multi16(address, index, buffer, BYTES_PER_WORD);
 *  }
 *
 *  return status;
 *
 * }
 *
 *
 * int32_t VL53L0X_write_dword16(uint8_t address, uint16_t index, uint32_t data)
 * {
 *  int32_t status = STATUS_OK;
 *  uint8_t  buffer[BYTES_PER_DWORD];
 *
 *  // Split 32-bit word into MS ... LS bytes
 *  buffer[0] = (uint8_t) (data >> 24);
 *  buffer[1] = (uint8_t)((data &  0x00FF0000) > 16);
 *  buffer[2] = (uint8_t)((data &  0x0000FF00) > 8);
 *  buffer[3] = (uint8_t) (data &  0x000000FF);
 *
 *  status = VL53L0X_write_multi16(address, index, buffer, BYTES_PER_DWORD);
 *
 *  return status;
 *
 * }
 *
 *
 * int32_t VL53L0X_read_byte16(uint8_t address, uint16_t index, uint8_t *pdata)
 * {
 *  int32_t status = STATUS_OK;
 *  int32_t cbyte_count = 1;
 *
 *  status = VL53L0X_read_multi16(address, index, pdata, cbyte_count);
 *
 *  return status;
 *
 * }
 *
 *
 * int32_t VL53L0X_read_word16(uint8_t address, uint16_t index, uint16_t *pdata)
 * {
 *  int32_t  status = STATUS_OK;
 *  uint8_t  buffer[BYTES_PER_WORD];
 *
 *  status = VL53L0X_read_multi16(address, index, buffer, BYTES_PER_WORD);
 * pdata = ((uint16_t)buffer[0]<<8) + (uint16_t)buffer[1];
 *
 *  return status;
 *
 * }
 *
 * int32_t VL53L0X_read_dword16(uint8_t address, uint16_t index, uint32_t *pdata)
 * {
 *  int32_t status = STATUS_OK;
 *  uint8_t  buffer[BYTES_PER_DWORD];
 *
 *  status = VL53L0X_read_multi16(address, index, buffer, BYTES_PER_DWORD);
 * pdata = ((uint32_t)buffer[0]<<24) + ((uint32_t)buffer[1]<<16) + ((uint32_t)buffer[2]<<8) + (uint32_t)buffer[3];
 *
 *  return status;
 *
 * }
 **************************************************************/

int32_t VL53L0X_platform_wait_us(int32_t wait_us) {
    int32_t status = STATUS_OK;
    float wait_ms = (float) wait_us / 1000.0f;

    /*
     * Use windows event handling to perform non-blocking wait.
     *
     * HANDLE hEvent = CreateEvent(0, TRUE, FALSE, 0);
     * WaitForSingleObject(hEvent, (int)(wait_ms + 0.5f));
     */

    return status;
}

int32_t VL53L0X_wait_ms(int32_t wait_ms) {
    int32_t status = STATUS_OK;

    /*
     * Use windows event handling to perform non-blocking wait.
     *
     * HANDLE hEvent = CreateEvent(0, TRUE, FALSE, 0);
     * WaitForSingleObject(hEvent, wait_ms);
     */

    return status;
}

int32_t VL53L0X_set_gpio(uint8_t level) {
    int32_t status = STATUS_OK;

    // status = VL53L0X_set_gpio_sv(level);

    return status;
}

int32_t VL53L0X_get_gpio(uint8_t* plevel) {
    int32_t status = STATUS_OK;

    return status;
}

int32_t VL53L0X_release_gpio(void) {
    int32_t status = STATUS_OK;

    return status;
}

int32_t VL53L0X_get_timer_frequency(int32_t* ptimer_freq_hz) {
    *ptimer_freq_hz = 0;
    return STATUS_FAIL;
}

int32_t VL53L0X_get_timer_value(int32_t* ptimer_count) {
    *ptimer_count = 0;
    return STATUS_FAIL;
}
