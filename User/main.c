#include <stdio.h>
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/KEY/key.h"
#include "./BSP/OV2640/ov2640.h"
#include "./BSP/DCMI/dcmi.h"
#include "./BSP/SRAM/sram.h"
#include "app_x-cube-ai.h"
#include "network.h"

AI_ALIGNED(4) ai_i8 ai_in[AI_NETWORK_IN_1_SIZE];
AI_ALIGNED(4) ai_i8 ai_out[AI_NETWORK_OUT_1_SIZE];

static volatile uint8_t g_frame_ready = 0;
static uint16_t *fb0, *fb1, *fb_ready;
static int g_buf_idx = 0;

/* dcmi.c 帧中断需要此函数, 双缓冲模式用 TC 中断, 此处空 */
void jpeg_data_process(void) {}

/* DMA 双缓冲 TC 中断回调: fb0/fb1 交替完成, 指向已完成的那块 */
void dma_rx_callback(void)
{
    fb_ready   = g_buf_idx ? fb1 : fb0;
    g_buf_idx  = !g_buf_idx;
    g_frame_ready = 1;
}

int main(void)
{
    uint8_t key;
    uint8_t msgbuf[30];
    uint32_t total_pixels, buf_bytes;

    HAL_Init();
    sys_stm32_clock_init(336, 8, 2, 7);
    usart_init(115200);
    delay_init(168);
    led_init();
    lcd_init();
    key_init();
    sram_init();

    lcd_show_string(30, 50, 200, 16, 16, "YOLOFace Real-Time", RED);
    lcd_show_string(30, 70, 200, 16, 16, "Loading AI...", RED);

    if (ai_init() != 0) {
        lcd_show_string(30, 90, 200, 16, 16, "AI Init FAIL", RED);
        while (1);
    }
    lcd_show_string(30, 90, 200, 16, 16, "AI Ready", RED);

    while (ov2640_init()) {
        lcd_show_string(30, 110, 240, 16, 16, "OV2640 ERROR", RED);
        delay_ms(200);
        LED0_TOGGLE();
    }
    lcd_show_string(30, 110, 200, 16, 16, "OV2640 OK", RED);
    ov2640_flash_intctrl();

    ov2640_rgb565_mode();
    ov2640_outsize_set(lcddev.width, lcddev.height);
    dcmi_init();

    total_pixels = lcddev.width * lcddev.height;
    buf_bytes    = total_pixels * sizeof(uint16_t);

    fb0 = (uint16_t *) SRAM_BASE_ADDR;
    fb1 = (uint16_t *)(SRAM_BASE_ADDR + buf_bytes);

    /* DMA 双缓冲: fb0/fb1 交替, TC 中断通知哪块就绪 */
    dcmi_rx_callback = dma_rx_callback;
    dcmi_dma_init((uint32_t)fb0, (uint32_t)fb1, total_pixels / 2,
                  DMA_MDATAALIGN_WORD, DMA_MINC_ENABLE);
    __HAL_DMA_ENABLE(&g_dma_dcmi_handle);
    DCMI->CR |= DCMI_CR_CAPTURE;

    lcd_show_string(30, 130, 200, 16, 16, "KEY_UP:Exit", RED);

    {
        uint32_t last_tick = HAL_GetTick();
        int fps = 0;

        while (1)
        {
            key = key_scan(0);
            if (key == WKUP_PRES) {
                DCMI->CR &= ~(DCMI_CR_CAPTURE);
                while (DCMI->CR & DCMI_CR_CAPTURE);
                __HAL_DMA_DISABLE(&g_dma_dcmi_handle);
                while (DMA2_Stream1->CR & DMA_SxCR_EN);
                break;
            }

            if (g_frame_ready) {
                g_frame_ready = 0;

                /* 计算帧率 */
                {
                    uint32_t now = HAL_GetTick();
                    uint32_t dt  = now - last_tick;
                    last_tick = now;
                    fps = (dt > 0) ? (int)(1000 / dt) : 0;
                }

                /* fb_ready 是已完成的完整帧, DMA 正在写另一块 */
                lcd_set_cursor(0, 0);
                lcd_write_ram_prepare();
                for (uint32_t i = 0; i < total_pixels; i++) {
                    LCD->LCD_RAM = fb_ready[i];
                }

                prepare_frame(fb_ready, lcddev.width, lcddev.height, ai_in);
                if (ai_run(ai_in, ai_out) == 0) {
                    post_process(ai_out);
                    draw_face_boxes();
                    sprintf((char *)msgbuf, "Faces:%d FPS:%d", get_detected_face_count(), fps);
                    lcd_show_string(0, 0, 200, 16, 16, (char *)msgbuf, RED);
                }
            }

            delay_ms(30);
        }
    }

    lcd_clear(WHITE);
    lcd_show_string(30, 50, 200, 16, 16, "Demo Exit", RED);
    while (1);
}
