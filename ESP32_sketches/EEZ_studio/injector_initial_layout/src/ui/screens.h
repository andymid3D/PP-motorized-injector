#ifndef EEZ_LVGL_UI_SCREENS_H
#define EEZ_LVGL_UI_SCREENS_H

#include <lvgl/lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _groups_t {
    lv_group_t *actual_mould_settings_labels;
} groups_t;

extern groups_t groups;

void ui_create_groups();

typedef struct _objects_t {
    lv_obj_t *main;
    lv_obj_t *mould_settings;
    lv_obj_t *common_settings;
    lv_obj_t *plunger_tip;
    lv_obj_t *plunger_tip__background_barrel;
    lv_obj_t *plunger_tip__background_barrel_interior;
    lv_obj_t *plunger_tip__plunger;
    lv_obj_t *plunger_tip__plunger_tip;
    lv_obj_t *plunger_tip__refill_hole;
    lv_obj_t *plunger_tip__obj0;
    lv_obj_t *obj0;
    lv_obj_t *obj0__background_barrel;
    lv_obj_t *obj0__background_barrel_interior;
    lv_obj_t *obj0__plunger;
    lv_obj_t *obj0__plunger_tip;
    lv_obj_t *obj0__refill_hole;
    lv_obj_t *obj0__obj0;
    lv_obj_t *obj1;
    lv_obj_t *obj1__test_plunger_slider;
    lv_obj_t *obj1__machine_state_text;
    lv_obj_t *obj2;
    lv_obj_t *obj2__background_barrel;
    lv_obj_t *obj2__background_barrel_interior;
    lv_obj_t *obj2__plunger;
    lv_obj_t *obj2__plunger_tip;
    lv_obj_t *obj2__refill_hole;
    lv_obj_t *obj2__obj0;
    lv_obj_t *obj3;
    lv_obj_t *obj3__test_plunger_slider;
    lv_obj_t *obj3__machine_state_text;
    lv_obj_t *obj4;
    lv_obj_t *obj4__title_mould_settings_page;
    lv_obj_t *obj4__actual_mould_settings_table_view;
    lv_obj_t *obj4__mould_name_label;
    lv_obj_t *obj4__mould_fill_speed;
    lv_obj_t *obj4__mould_fill_dist;
    lv_obj_t *obj4__mould_fill_accel;
    lv_obj_t *obj4__mould_hold_speed;
    lv_obj_t *obj4__mould_hold_dist;
    lv_obj_t *obj4__mould_hold_accel;
    lv_obj_t *obj4__mould_name_value;
    lv_obj_t *obj4__mould_fill_speed_value;
    lv_obj_t *obj4__mould_fill_dist_value;
    lv_obj_t *obj4__mould_fill_accel_value;
    lv_obj_t *obj4__mould_hold_speed_value;
    lv_obj_t *obj4__mould_hold_dist_value;
    lv_obj_t *obj4__mould_hold_accel_value;
    lv_obj_t *obj5;
    lv_obj_t *obj5__background_barrel;
    lv_obj_t *obj5__background_barrel_interior;
    lv_obj_t *obj5__plunger;
    lv_obj_t *obj5__plunger_tip;
    lv_obj_t *obj5__refill_hole;
    lv_obj_t *obj5__obj0;
    lv_obj_t *obj6;
    lv_obj_t *obj6__test_plunger_slider;
    lv_obj_t *obj6__machine_state_text;
    lv_obj_t *button_to_mould_settings;
    lv_obj_t *button_to_mould_settings_3;
    lv_obj_t *button_to_mould_settings_1;
    lv_obj_t *button_to_mould_settings_2;
} objects_t;

extern objects_t objects;

enum ScreensEnum {
    SCREEN_ID_MAIN = 1,
    SCREEN_ID_MOULD_SETTINGS = 2,
    SCREEN_ID_COMMON_SETTINGS = 3,
};

void create_screen_main();
void tick_screen_main();

void create_screen_mould_settings();
void tick_screen_mould_settings();

void create_screen_common_settings();
void tick_screen_common_settings();

void create_user_widget_plunger_and_tip(lv_obj_t *parent_obj, void *flowState, int startWidgetIndex);
void tick_user_widget_plunger_and_tip(void *flowState, int startWidgetIndex);

void create_user_widget_slider_encoder(lv_obj_t *parent_obj, void *flowState, int startWidgetIndex);
void tick_user_widget_slider_encoder(void *flowState, int startWidgetIndex);

void create_user_widget_actual_mould_settings_table(lv_obj_t *parent_obj, void *flowState, int startWidgetIndex);
void tick_user_widget_actual_mould_settings_table(void *flowState, int startWidgetIndex);

void create_user_widget_common_mould_settings_table(lv_obj_t *parent_obj, void *flowState, int startWidgetIndex);
void tick_user_widget_common_mould_settings_table(void *flowState, int startWidgetIndex);

void tick_screen_by_id(enum ScreensEnum screenId);
void tick_screen(int screen_index);

void create_screens();


#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_SCREENS_H*/