/*******************************************************************************
 Copyright (C) 2016, STMicroelectronics International N.V.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of STMicroelectronics nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
 IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/


#ifndef _VL53L1_PLATFORM_H_
#define _VL53L1_PLATFORM_H_

#include "vl53l1_types.h"
#include "vl53l1_platform_log.h"
#include "vl53l1_platform_user_defines.h"
#include "vl53l1_platform_user_config.h"

typedef struct VL53L1_Dev_t VL53L1_Dev_t;
typedef VL53L1_Dev_t* VL53L1_DEV;

#include "vl53l1_platform_user_data.h"
#include "vl53l1_error_codes.h"


VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count);
VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count);
VL53L1_Error VL53L1_WrByte(VL53L1_DEV Dev, uint16_t index, uint8_t data);
VL53L1_Error VL53L1_WrWord(VL53L1_DEV Dev, uint16_t index, uint16_t data);
VL53L1_Error VL53L1_WrDWord(VL53L1_DEV Dev, uint16_t index, uint32_t data);
VL53L1_Error VL53L1_RdByte(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata);
VL53L1_Error VL53L1_RdWord(VL53L1_DEV Dev, uint16_t index, uint16_t *pdata);
VL53L1_Error VL53L1_RdDWord(VL53L1_DEV Dev, uint16_t index, uint32_t *pdata);
VL53L1_Error VL53L1_WaitMs(VL53L1_DEV Dev, int32_t wait_ms);
VL53L1_Error VL53L1_WaitUs(VL53L1_DEV Dev, int32_t wait_us);
VL53L1_Error VL53L1_WaitValueMaskEx(VL53L1_DEV Dev, uint32_t timeout_ms, uint16_t index, uint8_t value, uint8_t mask, uint32_t poll_delay_ms);
VL53L1_Error VL53L1_GetTickCount(uint32_t *ptick_count_ms);

#endif