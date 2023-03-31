/**
 * Copyright (c) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file    common.h
 * @brief   Common header file for the BHI260/BHA260 examples
 *
 */

#ifndef _COMMON_H_
#define _COMMON_H_

#include <stdbool.h>

#include "../driver/bhy2.hpp"
#include "../driver/bhy2_klio.hpp"
#include "../driver/bhy2_pdr.hpp"
#include "../driver/bhy2_swim.hpp"
#include "../driver/bhy2_bsec.hpp"

#define BHY260_APP20_CS_PIN     COINES_SHUTTLE_PIN_7
#define BHY260_APP20_INT_PIN    COINES_SHUTTLE_PIN_21
#define BHY260_APP20_RESET_PIN  COINES_SHUTTLE_PIN_8
#define BHY260_APP30_CS_PIN     COINES_MINI_SHUTTLE_PIN_2_1
#define BHY260_APP30_INT_PIN    COINES_MINI_SHUTTLE_PIN_1_6
#define BHY260_APP30_RESET_PIN  COINES_MINI_SHUTTLE_PIN_2_6

#define BHY2_RD_WR_LEN          256

char *get_api_error(int8_t error_code);
char *get_sensor_error_text(uint8_t sensor_error);
char *get_sensor_name(uint8_t sensor_id);
float get_sensor_default_scaling(uint8_t sensor_id);
char *get_sensor_parse_format(uint8_t sensor_id);
char *get_sensor_axis_names(uint8_t sensor_id);
char *get_klio_error(bhy2_klio_driver_error_state_t error);

#endif /* _COMMON_H_ */
