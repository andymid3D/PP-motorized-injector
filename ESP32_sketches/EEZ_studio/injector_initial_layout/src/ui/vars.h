#ifndef EEZ_LVGL_UI_VARS_H
#define EEZ_LVGL_UI_VARS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// enum declarations

typedef enum {
    machine_state_cold = 0,
    machine_state_hot_not_homed = 1,
    machine_state_refill = 2,
    machine_state_compression = 3,
    machine_state_ready_to_inject = 4,
    machine_state_injecting = 5
} machine_state;

typedef enum {
    mould_params_options_mould_name = 0,
    mould_params_options_mould_fill_speed = 70,
    mould_params_options_fill_dist = 200,
    mould_params_options_fill_accel = 5000,
    mould_params_options_hold_speed = 10,
    mould_params_options_hold_dist = 5,
    mould_params_options_hold_accel = 3000
} mould_params_options;

// Flow global variables

enum FlowGlobalVariables {
    FLOW_GLOBAL_VARIABLE_MACHINE_STATE = 0,
    FLOW_GLOBAL_VARIABLE_PLUNGER_TIP_POSITION = 1,
    FLOW_GLOBAL_VARIABLE_MOULD_PARAMETERS_LIST = 2
};

// Native global variables



#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_VARS_H*/