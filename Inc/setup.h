/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
* Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "at32f4xx.h"
#include "config.h"

void MX_GPIO_Init(void);
void MX_TIM_Init(void);
void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void UART_Init(void);

#ifdef CONTROL_PPM
void PPM_Init();
void PPM_ISR_Callback();
void PPM_SysTick_Callback();
#endif

#ifdef CONTROL_PWM
void PWM_Init();
#endif

void Nunchuck_Init();
void Nunchuck_Read();
void consoleScope();
void consoleLog(char *message);
void setScopeChannel(uint8_t ch, int16_t val);

char* get_diagnostic_info();
void set_steer(int v);
void set_speed(int v);
void poweroff();
void update_timeout();

