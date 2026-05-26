

/**
  ******************************************************************************
  * @file    app_x-cube-ai.c
  * @author  X-CUBE-AI C code generator
  * @brief   AI program body - Face Detection with YOLOFace INT8
  ******************************************************************************
  */

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#if defined ( __ICCARM__ )
#elif defined ( __CC_ARM ) || ( __GNUC__ )
#endif

/* System headers */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>

#include "app_x-cube-ai.h"
#include "ai_datatypes_defines.h"
#include "network.h"
#include "network_data.h"

/* USER CODE BEGIN includes */
#include "./BSP/LCD/lcd.h"
/* USER CODE END includes */

/* IO buffers ----------------------------------------------------------------*/

#if !defined(AI_NETWORK_INPUTS_IN_ACTIVATIONS)
AI_ALIGNED(4) ai_i8 data_in_1[AI_NETWORK_IN_1_SIZE_BYTES];
ai_i8* data_ins[AI_NETWORK_IN_NUM] = {
data_in_1
};
#else
ai_i8* data_ins[AI_NETWORK_IN_NUM] = {
NULL
};
#endif

#if !defined(AI_NETWORK_OUTPUTS_IN_ACTIVATIONS)
AI_ALIGNED(4) ai_i8 data_out_1[AI_NETWORK_OUT_1_SIZE_BYTES];
ai_i8* data_outs[AI_NETWORK_OUT_NUM] = {
data_out_1
};
#else
ai_i8* data_outs[AI_NETWORK_OUT_NUM] = {
NULL
};
#endif

/* Activations buffers -------------------------------------------------------*/

AI_ALIGNED(32)
static uint8_t pool0[AI_NETWORK_DATA_ACTIVATION_1_SIZE];

ai_handle data_activations0[] = {pool0};

/* AI objects ----------------------------------------------------------------*/

static ai_handle network = AI_HANDLE_NULL;

static ai_buffer* ai_input;
static ai_buffer* ai_output;

static void ai_log_err(const ai_error err, const char *fct)
{
  /* USER CODE BEGIN log */
  (void)err;
  (void)fct;
  do {} while (1);
  /* USER CODE END log */
}

static int ai_boostrap(ai_handle *act_addr)
{
  ai_error err;

  err = ai_network_create_and_init(&network, act_addr, NULL);
  if (err.type != AI_ERROR_NONE) {
    ai_log_err(err, "ai_network_create_and_init");
    return -1;
  }

  ai_input = ai_network_inputs_get(network, NULL);
  ai_output = ai_network_outputs_get(network, NULL);

#if defined(AI_NETWORK_INPUTS_IN_ACTIVATIONS)
  for (int idx=0; idx < AI_NETWORK_IN_NUM; idx++) {
    data_ins[idx] = ai_input[idx].data;
  }
#else
  for (int idx=0; idx < AI_NETWORK_IN_NUM; idx++) {
      ai_input[idx].data = data_ins[idx];
  }
#endif

#if defined(AI_NETWORK_OUTPUTS_IN_ACTIVATIONS)
  for (int idx=0; idx < AI_NETWORK_OUT_NUM; idx++) {
    data_outs[idx] = ai_output[idx].data;
  }
#else
  for (int idx=0; idx < AI_NETWORK_OUT_NUM; idx++) {
    ai_output[idx].data = data_outs[idx];
  }
#endif

  return 0;
}

/* USER CODE BEGIN 2 */

typedef struct { float x, y, w, h, conf; } face_box_t;

static face_box_t faces[AI_NETWORK_OUT_1_WIDTH * AI_NETWORK_OUT_1_HEIGHT * 3];
static int face_count = 0;

static float sigmoid(float x)
{
    return 1.0f / (1.0f + expf(-x));
}

int ai_init(void)
{
    return ai_boostrap(data_activations0);
}

int ai_run(const void *in_data, void *out_data)
{
    ai_input[0].data  = AI_HANDLE_PTR(in_data);
    ai_output[0].data = AI_HANDLE_PTR(out_data);

    ai_i32 batch = ai_network_run(network, ai_input, ai_output);
    return (batch == 1) ? 0 : -1;
}

void prepare_frame(uint16_t *src, int src_w, int src_h, ai_i8 *dst)
{
    float sx = (float)src_w / AI_NETWORK_IN_1_WIDTH;
    float sy = (float)src_h / AI_NETWORK_IN_1_HEIGHT;

    for (int y = 0; y < AI_NETWORK_IN_1_HEIGHT; y++) {
        for (int x = 0; x < AI_NETWORK_IN_1_WIDTH; x++) {
            uint16_t c = src[(int)(y * sy) * src_w + (int)(x * sx)];
            int idx = (y * AI_NETWORK_IN_1_WIDTH + x) * AI_NETWORK_IN_1_CHANNEL;
            dst[idx]     = (ai_i8)(((c >> 11) & 0x1F) << 3);
            dst[idx + 1] = (ai_i8)(((c >> 5)  & 0x3F) << 2);
            dst[idx + 2] = (ai_i8)((c & 0x1F) << 3);
        }
    }
}

static float box_iou(face_box_t *a, face_box_t *b)
{
    float ax1 = a->x - a->w / 2, ay1 = a->y - a->h / 2;
    float ax2 = a->x + a->w / 2, ay2 = a->y + a->h / 2;
    float bx1 = b->x - b->w / 2, by1 = b->y - b->h / 2;
    float bx2 = b->x + b->w / 2, by2 = b->y + b->h / 2;

    float ix1 = ax1 > bx1 ? ax1 : bx1;
    float iy1 = ay1 > by1 ? ay1 : by1;
    float ix2 = ax2 < bx2 ? ax2 : bx2;
    float iy2 = ay2 < by2 ? ay2 : by2;
    float iw  = ix2 - ix1; if (iw < 0) iw = 0;
    float ih  = iy2 - iy1; if (ih < 0) ih = 0;
    float inter = iw * ih;

    float area_a = (ax2 - ax1) * (ay2 - ay1);
    float area_b = (bx2 - bx1) * (by2 - by1);
    float uni = area_a + area_b - inter;
    return (uni > 0) ? inter / uni : 0;
}

static void nms(face_box_t *boxes, int *count, float threshold)
{
    int i, j, keep;

    for (i = 0; i < *count - 1; i++) {
        for (j = i + 1; j < *count; j++) {
            if (boxes[j].conf > boxes[i].conf) {
                face_box_t tmp = boxes[i];
                boxes[i] = boxes[j];
                boxes[j] = tmp;
            }
        }
    }

    for (i = 0; i < *count; i++) {
        if (boxes[i].conf < 0) continue;
        for (j = i + 1; j < *count; j++) {
            if (boxes[j].conf < 0) continue;
            if (box_iou(&boxes[i], &boxes[j]) > threshold) {
                boxes[j].conf = -1;
            }
        }
    }

    keep = 0;
    for (i = 0; i < *count; i++) {
        if (boxes[i].conf >= 0) {
            boxes[keep++] = boxes[i];
        }
    }
    *count = keep;
}

void post_process(ai_i8 *out_data)
{
    const uint8_t anchors[3][2] = {{9,14},{12,17},{22,21}};
    const float s  = 0.14218327403068542f;
    const int8_t zp = -15;

    face_count = 0;

    for (int gy = 0; gy < AI_NETWORK_OUT_1_HEIGHT; gy++) {
        for (int gx = 0; gx < AI_NETWORK_OUT_1_WIDTH; gx++) {
            for (int b = 0; b < 3; b++) {
                int base = (gy * AI_NETWORK_OUT_1_WIDTH + gx) * AI_NETWORK_OUT_1_CHANNEL + b * 6;

                float conf = sigmoid((out_data[base + 4] - zp) * s);
                if (conf < 0.5f) continue;
                if (face_count >= (int)(sizeof(faces)/sizeof(faces[0]))) break;

                float tx = sigmoid((out_data[base]     - zp) * s);
                float ty = sigmoid((out_data[base + 1] - zp) * s);
                float tw = expf((out_data[base + 2] - zp) * s);
                float th = expf((out_data[base + 3] - zp) * s);

                faces[face_count].x = (tx + gx) / AI_NETWORK_OUT_1_WIDTH;
                faces[face_count].y = (ty + gy) / AI_NETWORK_OUT_1_HEIGHT;
                faces[face_count].w = tw * anchors[b][0] / AI_NETWORK_IN_1_WIDTH;
                faces[face_count].h = th * anchors[b][1] / AI_NETWORK_IN_1_HEIGHT;
                faces[face_count].conf = conf;
                face_count++;
            }
        }
    }

    nms(faces, &face_count, 0.5f);
}

void draw_face_boxes(void)
{
    for (int i = 0; i < face_count; i++) {
        int x1 = (int)((faces[i].x - faces[i].w / 2) * lcddev.width);
        int y1 = (int)((faces[i].y - faces[i].h / 2) * lcddev.height);
        int x2 = (int)((faces[i].x + faces[i].w / 2) * lcddev.width);
        int y2 = (int)((faces[i].y + faces[i].h / 2) * lcddev.height);

        if (x1 < 0) x1 = 0;
        if (y1 < 0) y1 = 0;
        if (x2 >= lcddev.width)  x2 = lcddev.width  - 1;
        if (y2 >= lcddev.height) y2 = lcddev.height - 1;

        /* 画3层加粗边框 */
        for (int t = -1; t <= 1; t++) {
            int tx1 = x1 + t; if (tx1 < 0) tx1 = 0;
            int ty1 = y1 + t; if (ty1 < 0) ty1 = 0;
            int tx2 = x2 - t; if (tx2 >= lcddev.width)  tx2 = lcddev.width  - 1;
            int ty2 = y2 - t; if (ty2 >= lcddev.height) ty2 = lcddev.height - 1;
            lcd_draw_rectangle(tx1, ty1, tx2, ty2, RED);
        }
    }
}

int get_detected_face_count(void)
{
    return face_count;
}
/* USER CODE END 2 */

/* Entry points --------------------------------------------------------------*/

void MX_X_CUBE_AI_Init(void)
{
    /* USER CODE BEGIN 5 */
    ai_boostrap(data_activations0);
    /* USER CODE END 5 */
}

void MX_X_CUBE_AI_Process(void)
{
    /* USER CODE BEGIN 6 */
    /* Use ai_init() + ai_run(in,out) + post_process(out) directly instead */
    /* USER CODE END 6 */
}
#ifdef __cplusplus
}
#endif
