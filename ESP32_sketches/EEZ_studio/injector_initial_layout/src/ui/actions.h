#ifndef EEZ_LVGL_UI_EVENTS_H
#define EEZ_LVGL_UI_EVENTS_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

extern void action_add_refill(lv_event_t * e);
extern void action_remove_refill(lv_event_t * e);
extern void action_clear_refills(lv_event_t * e);
extern void action_update_plunger_temperature(lv_event_t * e);
extern void action_update_refill_bands(lv_event_t * e);

#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_EVENTS_H*/