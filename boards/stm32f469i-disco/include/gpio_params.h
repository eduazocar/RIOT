/*
 * Copyright (c) 2021 Mesh4all <mesh4all.org>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @brief     Board specific configuration of direct mapped GPIOs
 *
 * @author      luisan00 <luisan00@hotmail.com>
 *
 */

#ifndef GPIO_PARAMS_H
#define GPIO_PARAMS_H

#include "board.h"
#include "saul/periph.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief    GPIO pin configuration
 *
 */
static const saul_gpio_params_t saul_gpio_params[] =
    {
        {.name = "LED1",
            .pin = LED0_PIN,
            .mode = GPIO_OUT,
            .flags = SAUL_GPIO_INIT_SET},
        {.name = "LED2",
            .pin = LED1_PIN,
            .mode = GPIO_OUT,
            .flags = SAUL_GPIO_INIT_SET},
        {.name = "LED3",
            .pin = LED2_PIN,
            .mode = GPIO_OUT,
            .flags = SAUL_GPIO_INIT_SET},
        {.name = "LED4",
            .pin = LED3_PIN,
            .mode = GPIO_OUT,
            .flags = SAUL_GPIO_INIT_SET},
        {.name = "BTN USER",
            .pin = BTN0_PIN,
            .mode = BTN0_MODE},
};

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* GPIO_PARAMS_H */
