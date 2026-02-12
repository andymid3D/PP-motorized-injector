#ifndef EEZ_LVGL_UI_STRUCTS_H
#define EEZ_LVGL_UI_STRUCTS_H

#include "eez-flow.h"

#include <stdint.h>
#include <stdbool.h>

#include "vars.h"

using namespace eez;

enum FlowStructures {
    FLOW_STRUCTURE_MOULD_PARAMETER = 16384,
    FLOW_STRUCTURE_REFILL = 16385,
    FLOW_STRUCTURE_PLUNGER_STATE = 16386
};

enum FlowArrayOfStructures {
    FLOW_ARRAY_OF_STRUCTURE_MOULD_PARAMETER = 81920,
    FLOW_ARRAY_OF_STRUCTURE_REFILL = 81921,
    FLOW_ARRAY_OF_STRUCTURE_PLUNGER_STATE = 81922
};

enum mould_parameterFlowStructureFields {
    FLOW_STRUCTURE_MOULD_PARAMETER_FIELD_PARAMETER_LABEL = 0,
    FLOW_STRUCTURE_MOULD_PARAMETER_FIELD_PARAMETER_VALUE = 1,
    FLOW_STRUCTURE_MOULD_PARAMETER_NUM_FIELDS
};

enum refillFlowStructureFields {
    FLOW_STRUCTURE_REFILL_FIELD_SIZE = 0,
    FLOW_STRUCTURE_REFILL_FIELD_TEMPERATURE = 1,
    FLOW_STRUCTURE_REFILL_FIELD_REFILL_TIME = 2,
    FLOW_STRUCTURE_REFILL_NUM_FIELDS
};

enum plunger_stateFlowStructureFields {
    FLOW_STRUCTURE_PLUNGER_STATE_FIELD_REFILLS = 0,
    FLOW_STRUCTURE_PLUNGER_STATE_FIELD_TEMPERATURE = 1,
    FLOW_STRUCTURE_PLUNGER_STATE_FIELD_CURRENT_BARREL_CAPACITY = 2,
    FLOW_STRUCTURE_PLUNGER_STATE_FIELD_MAX_BARREL_CAPACITY = 3,
    FLOW_STRUCTURE_PLUNGER_STATE_NUM_FIELDS
};

struct mould_parameterValue {
    Value value;
    
    mould_parameterValue() {
        value = Value::makeArrayRef(FLOW_STRUCTURE_MOULD_PARAMETER_NUM_FIELDS, FLOW_STRUCTURE_MOULD_PARAMETER, 0);
    }
    
    mould_parameterValue(Value value) : value(value) {}
    
    operator Value() const { return value; }
    
    operator bool() const { return value.isArray(); }
    
    const char *parameter_label() {
        return value.getArray()->values[FLOW_STRUCTURE_MOULD_PARAMETER_FIELD_PARAMETER_LABEL].getString();
    }
    void parameter_label(const char *parameter_label) {
        value.getArray()->values[FLOW_STRUCTURE_MOULD_PARAMETER_FIELD_PARAMETER_LABEL] = StringValue(parameter_label);
    }
    
    Value parameter_value() {
        return value.getArray()->values[FLOW_STRUCTURE_MOULD_PARAMETER_FIELD_PARAMETER_VALUE];
    }
    void parameter_value(Value parameter_value) {
        value.getArray()->values[FLOW_STRUCTURE_MOULD_PARAMETER_FIELD_PARAMETER_VALUE] = parameter_value;
    }
};

typedef ArrayOf<mould_parameterValue, FLOW_ARRAY_OF_STRUCTURE_MOULD_PARAMETER> ArrayOfmould_parameterValue;
struct refillValue {
    Value value;
    
    refillValue() {
        value = Value::makeArrayRef(FLOW_STRUCTURE_REFILL_NUM_FIELDS, FLOW_STRUCTURE_REFILL, 0);
    }
    
    refillValue(Value value) : value(value) {}
    
    operator Value() const { return value; }
    
    operator bool() const { return value.isArray(); }
    
    int size() {
        return value.getArray()->values[FLOW_STRUCTURE_REFILL_FIELD_SIZE].getInt();
    }
    void size(int size) {
        value.getArray()->values[FLOW_STRUCTURE_REFILL_FIELD_SIZE] = IntegerValue(size);
    }
    
    float temperature() {
        return value.getArray()->values[FLOW_STRUCTURE_REFILL_FIELD_TEMPERATURE].getFloat();
    }
    void temperature(float temperature) {
        value.getArray()->values[FLOW_STRUCTURE_REFILL_FIELD_TEMPERATURE] = FloatValue(temperature);
    }
    
    int refillTime() {
        return value.getArray()->values[FLOW_STRUCTURE_REFILL_FIELD_REFILL_TIME].getInt();
    }
    void refillTime(int refillTime) {
        value.getArray()->values[FLOW_STRUCTURE_REFILL_FIELD_REFILL_TIME] = IntegerValue(refillTime);
    }
};

typedef ArrayOf<refillValue, FLOW_ARRAY_OF_STRUCTURE_REFILL> ArrayOfrefillValue;
struct plunger_stateValue {
    Value value;
    
    plunger_stateValue() {
        value = Value::makeArrayRef(FLOW_STRUCTURE_PLUNGER_STATE_NUM_FIELDS, FLOW_STRUCTURE_PLUNGER_STATE, 0);
    }
    
    plunger_stateValue(Value value) : value(value) {}
    
    operator Value() const { return value; }
    
    operator bool() const { return value.isArray(); }
    
    ArrayOfrefillValue refills() {
        return value.getArray()->values[FLOW_STRUCTURE_PLUNGER_STATE_FIELD_REFILLS];
    }
    void refills(ArrayOfrefillValue refills) {
        value.getArray()->values[FLOW_STRUCTURE_PLUNGER_STATE_FIELD_REFILLS] = refills.value;
    }
    
    float temperature() {
        return value.getArray()->values[FLOW_STRUCTURE_PLUNGER_STATE_FIELD_TEMPERATURE].getFloat();
    }
    void temperature(float temperature) {
        value.getArray()->values[FLOW_STRUCTURE_PLUNGER_STATE_FIELD_TEMPERATURE] = FloatValue(temperature);
    }
    
    float current_barrel_capacity() {
        return value.getArray()->values[FLOW_STRUCTURE_PLUNGER_STATE_FIELD_CURRENT_BARREL_CAPACITY].getFloat();
    }
    void current_barrel_capacity(float current_barrel_capacity) {
        value.getArray()->values[FLOW_STRUCTURE_PLUNGER_STATE_FIELD_CURRENT_BARREL_CAPACITY] = FloatValue(current_barrel_capacity);
    }
    
    float max_barrel_capacity() {
        return value.getArray()->values[FLOW_STRUCTURE_PLUNGER_STATE_FIELD_MAX_BARREL_CAPACITY].getFloat();
    }
    void max_barrel_capacity(float max_barrel_capacity) {
        value.getArray()->values[FLOW_STRUCTURE_PLUNGER_STATE_FIELD_MAX_BARREL_CAPACITY] = FloatValue(max_barrel_capacity);
    }
};

typedef ArrayOf<plunger_stateValue, FLOW_ARRAY_OF_STRUCTURE_PLUNGER_STATE> ArrayOfplunger_stateValue;

#endif /*EEZ_LVGL_UI_STRUCTS_H*/