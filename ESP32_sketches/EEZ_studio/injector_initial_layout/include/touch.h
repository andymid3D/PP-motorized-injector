/*******************************************************************************
 * Touch libraries:
 * FT6X36: https://github.com/strange-v/FT6X36.git
 * GT911: https://github.com/TAMCTec/gt911-arduino.git
 * XPT2046: https://github.com/PaulStoffregen/XPT2046_Touchscreen.git
 ******************************************************************************/

 #define TOUCH_GT911
 #define TOUCH_GT911_SCL 20//20
 #define TOUCH_GT911_SDA 19//19
 #define TOUCH_GT911_INT -1//-1
 #define TOUCH_GT911_RST -1//38
 #define TOUCH_GT911_ROTATION ROTATION_NORMAL // Rotation 90 degrees clockwise
 // #define TOUCH_SWAP_XY
 #define TOUCH_MAP_X1 0//480
 #define TOUCH_MAP_X2 800
 #define TOUCH_MAP_Y1 480//272
 #define TOUCH_MAP_Y2 0

int touch_last_x = 0, touch_last_y = 0;

#include <Wire.h>
#include <TAMC_GT911.h>
TAMC_GT911 ts = TAMC_GT911(TOUCH_GT911_SDA, TOUCH_GT911_SCL, TOUCH_GT911_INT, TOUCH_GT911_RST, max(TOUCH_MAP_X1, TOUCH_MAP_X2), max(TOUCH_MAP_Y1, TOUCH_MAP_Y2));

void touch_init()
{

  Wire.begin(TOUCH_GT911_SDA, TOUCH_GT911_SCL);
  ts.begin();
  ts.setRotation(TOUCH_GT911_ROTATION);
}

bool touch_has_signal()
{
  return true;
}

bool touch_touched()
{
  ts.read();
  if (ts.isTouched)
  {
#if defined(TOUCH_SWAP_XY)
    touch_last_y = map(ts.points[0].x, TOUCH_MAP_X1, TOUCH_MAP_X2, 0, lcd.width() - 1);
    touch_last_x = map(ts.points[0].y, TOUCH_MAP_Y1, TOUCH_MAP_Y2, 0, lcd.height() - 1);

    Serial.print("Touch: INV [");
    Serial.print(ts.points[0].x) ;
    Serial.print(", ");
    Serial.print(ts.points[0].y);
    Serial.print(",] => [");
    Serial.print(touch_last_x) ;
    Serial.print(", ");
    Serial.print(touch_last_y);
    Serial.println("]");

#else
    //touch_last_x = map(ts.points[0].x, TOUCH_MAP_X1, TOUCH_MAP_X2, 0, lcd.width() - 1);
    //touch_last_y = map(ts.points[0].y, TOUCH_MAP_Y1, TOUCH_MAP_Y2, 0, lcd.height() - 1);

    touch_last_x = ts.points[0].x;
    touch_last_y = ts.points[0].y;
#endif

    return true;
  }
  else
  {
    return false;
  }
}

bool touch_released()
{
  return true;
}