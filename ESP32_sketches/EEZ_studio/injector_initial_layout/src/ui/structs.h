#ifndef EEZ_LVGL_UI_STRUCTS_H
#define EEZ_LVGL_UI_STRUCTS_H

#include "eez-flow.h"


#if defined(EEZ_FOR_LVGL)

#include <stdint.h>
#include <stdbool.h>

#include "vars.h"

using namespace eez;

enum FlowStructures {
    FLOW_STRUCTURE_MOULD_PARAMETER = 16384
};

enum FlowArrayOfStructures {
    FLOW_ARRAY_OF_STRUCTURE_MOULD_PARAMETER = 81920
};

enum mould_parameterFlowStructureFields {
    FLOW_STRUCTURE_MOULD_PARAMETER_FIELD_PARAMETER_LABEL = 0,
    FLOW_STRUCTURE_MOULD_PARAMETER_FIELD_PARAMETER_VALUE = 1,
    FLOW_STRUCTURE_MOULD_PARAMETER_NUM_FIELDS
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


#endif

#endif /*EEZ_LVGL_UI_STRUCTS_H*/
