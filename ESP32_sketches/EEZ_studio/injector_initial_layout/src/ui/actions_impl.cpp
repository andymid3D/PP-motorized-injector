/**
 * Implementation of native actions for the injector HMI
 * This file is NOT auto-generated and should be compiled with your main sketch
 */

#include "actions.h"
#include "screens.h"
#include "ui.h"
#include <cstdlib>
#include <ctime>

// Maximum number of refills supported
#define MAX_REFILLS 16

// Barrel visualization constants
constexpr float BARREL_HEIGHT_PX = 793.0f;
constexpr float BARREL_CAPACITY_MM_CONST = 250.0f;
constexpr float SCALE_FACTOR = BARREL_HEIGHT_PX / BARREL_CAPACITY_MM_CONST;

extern const float BARREL_CAPACITY_MM = BARREL_CAPACITY_MM_CONST;

// Refill data structure
struct Refill {
    float size;         // Size in mm
    float temperature;  // Temperature in Celsius
    int refillTime;     // Unix timestamp
};

// Plunger state (in-memory storage)
struct PlungerState {
    Refill refills[MAX_REFILLS];
    int refillCount;
    float temperature;
    float currentBarrelLevel; // in mm
};

static PlungerState plungerState = {
    .refillCount = 0,
    .temperature = 0.0f,
    .currentBarrelLevel = 0.0f
};

extern "C" {

/**
 * Add a new refill to the plunger state
 * Triggered by the "Add Refill" button
 */
void action_add_refill(lv_event_t * e) {

    Serial.println("Add Refill action triggered. Current refill count: " + String(plungerState.refillCount));

    if (plungerState.refillCount >= MAX_REFILLS) {
        return;
    }
    
    lv_obj_t * textarea = objects.obj1__refill_size_input;
    const char * size_text = lv_textarea_get_text(textarea);
    float size = atof(size_text);
    
    if (size <= 0 || plungerState.currentBarrelLevel + size > BARREL_CAPACITY_MM) {
        return;
    }
    
    Refill& refill = plungerState.refills[plungerState.refillCount];
    refill.size = size;
    refill.temperature = plungerState.temperature;
    refill.refillTime = time(nullptr);
    
    plungerState.refillCount++;
    plungerState.currentBarrelLevel += size;
    
    action_update_refill_bands(nullptr);
    
    lv_textarea_set_text(textarea, "50");
}

/**
 * Remove a refill from the plunger state by index
 */
void action_remove_refill(lv_event_t * e) {
    if (e && e->user_data) {
        int index = static_cast<int>(reinterpret_cast<intptr_t>(e->user_data));
        
        if (index >= 0 && index < plungerState.refillCount) {
            for (int i = index; i < plungerState.refillCount - 1; i++) {
                plungerState.refills[i] = plungerState.refills[i + 1];
            }
            plungerState.refillCount--;
            action_update_refill_bands(nullptr);
        }
    }
}

/**
 * Clear all refills from the plunger state
 */
void action_clear_refills(lv_event_t * e) {
    plungerState.refillCount = 0;
    action_update_refill_bands(nullptr);
}

/**
 * Update the current plunger temperature
 */
void action_update_plunger_temperature(lv_event_t * e) {
    if (e && e->user_data) {
        float* temp_ptr = static_cast<float*>(e->user_data);
        plungerState.temperature = *temp_ptr;
    }
}

/**
 * Update the refill band visualization based on current refills
 */
void action_update_refill_bands(lv_event_t * e) {
    lv_obj_t* bands[MAX_REFILLS] = {
        objects.plunger_tip__refill_band_0,
        objects.plunger_tip__refill_band_1,
        objects.plunger_tip__refill_band_2,
        objects.plunger_tip__refill_band_3,
        objects.plunger_tip__refill_band_4,
        objects.plunger_tip__refill_band_5,
        objects.plunger_tip__refill_band_6,
        objects.plunger_tip__refill_band_7,
        objects.plunger_tip__refill_band_8,
        objects.plunger_tip__refill_band_9,
        objects.plunger_tip__refill_band_10,
        objects.plunger_tip__refill_band_11,
        objects.plunger_tip__refill_band_12,
        objects.plunger_tip__refill_band_13,
        objects.plunger_tip__refill_band_14,
        objects.plunger_tip__refill_band_15
    };
    
    float current_y = BARREL_HEIGHT_PX;
    
    for (int i = 0; i < MAX_REFILLS; i++) {
        if (i < plungerState.refillCount) {
            float size_mm = plungerState.refills[i].size;
            float height_px = size_mm * SCALE_FACTOR;
            
            current_y -= height_px;
            
            lv_obj_set_y(bands[i], static_cast<lv_coord_t>(current_y));
            lv_obj_set_height(bands[i], static_cast<lv_coord_t>(height_px));
            lv_obj_clear_flag(bands[i], LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(bands[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}

} // extern "C"
