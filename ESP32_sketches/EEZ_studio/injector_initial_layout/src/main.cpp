/**
 * Motorized Injector HMI - EEZ Studio LVGL Project
 * Main PlatformIO sketch for ESP32 with Elecrow 5" HMI display
 */

#include <Arduino.h>
#include <lvgl.h>
#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>

// EEZ Studio generated UI files
#include "ui/ui.h"
#include "ui/structs.h"
#include "ui/vars.h"
#include "ui/eez-flow.h"

// Import BARREL_CAPACITY_MM from actions
extern const float BARREL_CAPACITY_MM;

// Physical display dimensions:
#define TFT_WIDTH 800
#define TFT_HEIGHT 480
// Screen is rotated 90 degrees clockwise, so width and height are swapped
// We should change LCD hardware using:
//   lcd.setRotation(1);
// LVGL using:
//   lv_display_set_rotation(display, LV_DISPLAY_ROTATION_90);
// Touch driver in touch.h using:
//   #define TOUCH_GT911_ROTATION = ROTATION_RIGHT


#define TFT_BL 2

// LovyanGFX display configuration for Elecrow 5" RGB display
class LGFX : public lgfx::LGFX_Device
{
public:
  lgfx::Bus_RGB     _bus_instance;
  lgfx::Panel_RGB   _panel_instance;

  LGFX(void)
  {
    {
      auto cfg = _bus_instance.config();
      cfg.panel = &_panel_instance;
      
      cfg.pin_d0  = GPIO_NUM_8;   // B0
      cfg.pin_d1  = GPIO_NUM_3;   // B1
      cfg.pin_d2  = GPIO_NUM_46;  // B2
      cfg.pin_d3  = GPIO_NUM_9;   // B3
      cfg.pin_d4  = GPIO_NUM_1;   // B4
      
      cfg.pin_d5  = GPIO_NUM_5;   // G0
      cfg.pin_d6  = GPIO_NUM_6;   // G1
      cfg.pin_d7  = GPIO_NUM_7;   // G2
      cfg.pin_d8  = GPIO_NUM_15;  // G3
      cfg.pin_d9  = GPIO_NUM_16;  // G4
      cfg.pin_d10 = GPIO_NUM_4;   // G5
      
      cfg.pin_d11 = GPIO_NUM_45;  // R0
      cfg.pin_d12 = GPIO_NUM_48;  // R1
      cfg.pin_d13 = GPIO_NUM_47;  // R2
      cfg.pin_d14 = GPIO_NUM_21;  // R3
      cfg.pin_d15 = GPIO_NUM_14;  // R4

      cfg.pin_henable = GPIO_NUM_40;
      cfg.pin_vsync   = GPIO_NUM_41;
      cfg.pin_hsync   = GPIO_NUM_39;
      cfg.pin_pclk    = GPIO_NUM_0;
      cfg.freq_write  = 15000000;

      cfg.hsync_polarity    = 0;
      cfg.hsync_front_porch = 8;
      cfg.hsync_pulse_width = 4;
      cfg.hsync_back_porch  = 43;
      
      cfg.vsync_polarity    = 0;
      cfg.vsync_front_porch = 8;
      cfg.vsync_pulse_width = 4;
      cfg.vsync_back_porch  = 12;

      cfg.pclk_active_neg   = 1;
      cfg.de_idle_high      = 0;
      cfg.pclk_idle_high    = 0;

      _bus_instance.config(cfg);
    }
    {
      auto cfg = _panel_instance.config();
      cfg.memory_width  = TFT_WIDTH;
      cfg.memory_height = TFT_HEIGHT;
      cfg.panel_width   = TFT_WIDTH;
      cfg.panel_height  = TFT_HEIGHT;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      _panel_instance.config(cfg);
    }
    _panel_instance.setBus(&_bus_instance);
    setPanel(&_panel_instance);
  }
};

LGFX lcd;


// Touch panel configuration
// Should be after lcd declaration as touch.h uses lcd for mapping touch coordinates
#include "touch.h"

// Display flushing callback for LVGL
void my_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  lv_draw_sw_rgb565_swap(px_map, w*h);
  lcd.pushImageDMA(area->x1, area->y1, w, h, (uint16_t *)px_map);
  lv_disp_flush_ready(disp);
}

uint32_t my_tick_cb() {
  return (esp_timer_get_time() / 1000LL);
}

void my_touch_read_cb(lv_indev_t * drv, lv_indev_data_t * data) { 
  
  if (touch_has_signal())
  {
    if (touch_touched())
    {
      data->state = LV_INDEV_STATE_PRESSED;

      /*Set the coordinates*/
      data->point.x = touch_last_x;
      data->point.y = touch_last_y;
    }
    else if (touch_released())
    {
      data->state = LV_INDEV_STATE_RELEASED;
    }
  }
  else
  {
    data->state = LV_INDEV_STATE_RELEASED;
  }
  delay(15);

}

void setup() {
  Serial.begin(115200);
  Serial.println("Motorized Injector HMI Starting...");

  // Initialize backlight
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  Serial.println("Backlight ON");

  // Initialize display
  lcd.init();
  lcd.setRotation(1);  // set rotation to match LVGL's rotation
  lcd.fillScreen(TFT_BLACK);
  delay(200);

Serial.print("Display ");
Serial.print(lcd.width());
Serial.print("x");
Serial.println(lcd.height());

  // Initialize LVGL
  lv_init();
  
  /*Set millisecond-based tick source for LVGL so that it can track time.*/
  lv_tick_set_cb(my_tick_cb);

  /*Create a display where screens and widgets can be added*/
  lv_display_t *display = lv_display_create(TFT_WIDTH, TFT_HEIGHT);

  /*Add rendering buffers to the screen.
    *Here adding a smaller partial buffer assuming 16-bit (RGB565 color format)*/
  static uint8_t buf[TFT_WIDTH * TFT_HEIGHT / 10 * 2]; /* x2 because of 16-bit color depth */
  lv_display_set_buffers(display, buf, NULL, sizeof(buf), LV_DISPLAY_RENDER_MODE_PARTIAL);


  /*Add a callback that can flush the content from `buf` when it has been rendered*/
  lv_display_set_flush_cb(display, my_flush_cb);


  // Set software rotation to portrait (90 degrees)
  lv_display_set_rotation(display, LV_DISPLAY_ROTATION_90);

  /*Create an input device for touch handling*/
  lv_indev_t * indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, my_touch_read_cb);

  
  // Set as default display for LVGL 9.3
  //lv_display_set_default(display);
  
  Serial.println("Display initialized");

  ui_init();

  // Sync max barrel capacity from native constant into EEZ global state
  plunger_stateValue plungerStateValue(eez::flow::getGlobalVariable(FLOW_GLOBAL_VARIABLE_PLUNGER_STATE));
  if (plungerStateValue) {
    plungerStateValue.max_barrel_capacity(BARREL_CAPACITY_MM);
  }


  // Initialize touch
  touch_init();
  Serial.println("Touch initialized");
}

void loop() {
  lv_timer_handler();
  ui_tick();
  delay(5);
}
