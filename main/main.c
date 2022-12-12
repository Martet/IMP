#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "lvgl.h"

static const char *TAG = "PONG";

#define I2C_HOST  0

#define LCD_PIXEL_CLOCK_HZ    (400 * 1000)
#define PIN_NUM_SDA           21
#define PIN_NUM_SCL           22
#define PIN_NUM_RST           -1
#define I2C_HW_ADDR           0x3C

// The pixel number in horizontal and vertical
#define LCD_H_RES              128
#define LCD_V_RES              64
// Bit number used to represent command and parameter
#define LCD_CMD_BITS           8
#define LCD_PARAM_BITS         8

#define LVGL_TICK_PERIOD_MS    2
#define GAME_TICK_PERIOD_MS    25
#define DRAW_TICK_PERIOD_MS    10

lv_obj_t *paddle1, *paddle2, *ball;
lv_obj_t *score1Label, *score2Label;
adc_oneshot_unit_handle_t adc1_handle;
int ballDirectionX = 2;
int ballDirectionY = 1;
int score1 = 0, score2 = 0;
int oldScore1 = 0, oldScore2 = 0;
int paddle1Y = 22, paddle2Y = 22;
int ballX = 60, ballY = 28;

int map(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

static void lvgl_set_px_cb(lv_disp_drv_t *disp_drv, uint8_t *buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
                                   lv_color_t color, lv_opa_t opa)
{
    uint16_t byte_index = x + (( y >> 3 ) * buf_w);
    uint8_t  bit_index  = y & 0x7;

    if ((color.full == 0) && (LV_OPA_TRANSP != opa)) {
        buf[byte_index] |= (1 << bit_index);
    } else {
        buf[byte_index] &= ~(1 << bit_index);
    }
}

static void lvgl_rounder(lv_disp_drv_t *disp_drv, lv_area_t *area)
{
    area->y1 = area->y1 & (~0x7);
    area->y2 = area->y2 | 0x7;
}

static void increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static void draw_tick(void *arg)
{
    lv_obj_set_y(paddle1, paddle1Y);
    lv_obj_set_y(paddle2, paddle2Y);
    lv_obj_set_pos(ball, ballX, ballY);

    char buf[4];
    if (score1 != oldScore1) {
        lv_snprintf(buf, sizeof(buf), "%d", score1);
        lv_label_set_text(score1Label, buf);
        oldScore1 = score1;
    }
    if (score2 != oldScore2) {
        lv_snprintf(buf, sizeof(buf), "%d", score2);
        lv_label_set_text(score2Label, buf);
        oldScore2 = score2;
    }
}

void get_hit_direction(int ballY, int paddleY, int *direction)
{
    int hitDirection = (ballY + 4 - paddleY) / 5 - 2;
    if (hitDirection != 0)
        *direction = hitDirection;
}

static void game_tick()
{
    // Set player 1 paddle based on joystick
    int y;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &y));
    y = map(y + 300, 0, 4095, -5, 5);
    paddle1Y += y;
    if (paddle1Y < 0) {
        paddle1Y = 0;
    } else if (paddle1Y > 44) {
        paddle1Y = 44;
    }

    // Set player 2 paddle based on ball position
    if (paddle2Y > 0 && ballY + 4 < paddle2Y + 10) {
        paddle2Y -= 1;
    } else if (paddle2Y < 44 && ballY + 4 > paddle2Y + 10) {
        paddle2Y += 1;
    }

    // Change ball directions if it hits a paddle
    if (ballX < 6) {
        ballDirectionX = -ballDirectionX;
        if (ballY + 4 < paddle1Y + 2 || ballY + 4 > paddle1Y + 18) {
            // Player 2 scores
            ballX = 60;
            ballY = 28;
            score2++;
            return;
        }
        get_hit_direction(ballY, paddle1Y, &ballDirectionY);
        ballX = 6;
    } else if (ballX > 118) {
        ballDirectionX = -ballDirectionX;
        if (ballY + 4 < paddle2Y + 2 || ballY + 4 > paddle2Y + 18) {
            // Player 1 scores
            ballX = 60;
            ballY = 28;
            score1++;
            return;
        }
        get_hit_direction(ballY, paddle2Y, &ballDirectionY);
        ballX = 118;
    }
    if (ballY <= 0 || ballY >= 56) {
        ballDirectionY = -ballDirectionY;
    }

    ballX += ballDirectionX;
    ballY += ballDirectionY;
}

void app_main(void)
{
    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions

    ESP_LOGI(TAG, "Initialize I2C bus");
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_NUM_SDA,
        .scl_io_num = PIN_NUM_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = LCD_PIXEL_CLOCK_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_HOST, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_HOST, I2C_MODE_MASTER, 0, 0, 0));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = I2C_HW_ADDR,
        .control_phase_bytes = 1,               // According to SSD1306 datasheet
        .dc_bit_offset = 6,                     // According to SSD1306 datasheet
        .lcd_cmd_bits = LCD_CMD_BITS,   // According to SSD1306 datasheet
        .lcd_param_bits = LCD_CMD_BITS, // According to SSD1306 datasheet
        .on_color_trans_done = notify_lvgl_flush_ready,
        .user_ctx = &disp_drv,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_HOST, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = PIN_NUM_RST,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = malloc(LCD_H_RES * 20 * sizeof(lv_color_t));
    assert(buf1);
    lv_color_t *buf2 = malloc(LCD_H_RES * 20 * sizeof(lv_color_t));
    assert(buf2);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LCD_H_RES * 20);

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    disp_drv.rounder_cb = lvgl_rounder;
    disp_drv.set_px_cb = lvgl_set_px_cb;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));

    ESP_LOGI(TAG, "Install game timer");
    const esp_timer_create_args_t game_timer_args = {
        .callback = &game_tick,
        .name = "game_tick"
    };
    esp_timer_handle_t game_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&game_timer_args, &game_timer));

    ESP_LOGI(TAG, "Install drawing timer");
    const esp_timer_create_args_t draw_timer_args = {
        .callback = &draw_tick,
        .name = "draw_tick"
    };
    esp_timer_handle_t draw_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&draw_timer_args, &draw_timer));

    ESP_LOGI(TAG, "Initialize ADC");
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t adc1_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &adc1_config));

    ESP_LOGI(TAG, "Create drawing objects");
    static lv_style_t style;
    lv_style_init(&style);
    lv_style_set_bg_color(&style, lv_color_black());
    lv_style_set_bg_opa(&style, LV_OPA_COVER);
    lv_style_set_border_color(&style, lv_color_black());

    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    paddle1 = lv_obj_create(scr);
    paddle2 = lv_obj_create(scr);
    ball = lv_obj_create(scr);
    lv_obj_add_style(paddle1, &style, 0);
    lv_obj_add_style(paddle2, &style, 0);
    lv_obj_add_style(ball, &style, 0);
    lv_obj_set_size(paddle1, 5, 20);
    lv_obj_set_size(paddle2, 5, 20);
    lv_obj_set_size(ball, 8, 8);
    lv_obj_set_pos(paddle1, 0, paddle1Y);
    lv_obj_set_pos(paddle2, 123, paddle2Y);
    lv_obj_set_pos(ball, ballX, ballY);

    score1Label = lv_label_create(scr);
    lv_label_set_text(score1Label, "0");
    lv_obj_align(score1Label, LV_ALIGN_TOP_MID, -30, 0);
    score2Label = lv_label_create(scr);
    lv_label_set_text(score2Label, "0");
    lv_obj_align(score2Label, LV_ALIGN_TOP_MID, 30, 0);

    ESP_LOGI(TAG, "Start timers");
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));
    ESP_ERROR_CHECK(esp_timer_start_periodic(game_timer, GAME_TICK_PERIOD_MS * 1000));
    ESP_ERROR_CHECK(esp_timer_start_periodic(draw_timer, DRAW_TICK_PERIOD_MS * 1000));

    while (1) {
        // raise the task priority of LVGL and/or reduce the handler period can improve the performance
        vTaskDelay(pdMS_TO_TICKS(10));
        // The task running lv_timer_handler should have lower priority than that running `lv_tick_inc`
        lv_timer_handler();
    }
}
