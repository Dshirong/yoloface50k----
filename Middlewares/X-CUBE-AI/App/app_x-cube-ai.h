
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_AI_H
#define __APP_AI_H
#ifdef __cplusplus
extern "C" {
#endif
/**
  ******************************************************************************
  * @file    app_x-cube-ai.h
  * @author  X-CUBE-AI C code generator
  * @brief   AI entry function definitions
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "ai_platform.h"

void MX_X_CUBE_AI_Init(void);
void MX_X_CUBE_AI_Process(void);
/* USER CODE BEGIN includes */
int  ai_init(void);
int  ai_run(const void *in_data, void *out_data);
void prepare_frame(uint16_t *src, int src_w, int src_h, ai_i8 *dst);
void post_process(ai_i8 *out_data);
void draw_face_boxes(void);
int  get_detected_face_count(void);
/* USER CODE END includes */
#ifdef __cplusplus
}
#endif
#endif /*__STMicroelectronics_X-CUBE-AI_10_2_0_H */
