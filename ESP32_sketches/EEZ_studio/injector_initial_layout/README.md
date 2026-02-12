# Motorized Injector HMI Interface

## Project Overview
This is an **EEZ Studio LVGL-based** graphical user interface project for a motorized injection machine control system targeting the **ESP32 platform**.

## Hardware Platform
- **Target Device**: Elecrow 5" HMI ESP32 RGB Touch Screen
- **Display Drivers**: ILI6122 & ILI5960
- **Display Specifications**: 
  - Resolution: 480×800 pixels
  - Color Format: BGR
  - Theme: Dark mode enabled

## Framework & Technology Stack
- **UI Framework**: LVGL (Light and Versatile Graphics Library) version 8.3
- **Project Type**: LVGL with Flow support
- **Development Tool**: EEZ Studio
- **Programming Language**: C/C++
- **Build Output**: Generated C code in `src/ui/` directory

## Application Purpose
This system controls a **precision injection molding machine** with real-time control and monitoring capabilities.

### Machine States
The system manages six distinct operational states:

1. **Cold** (0) - Initial/idle state
2. **Hot (Not Homed)** (1) - Heated but not calibrated
3. **Refill** (2) - Material loading phase
4. **Compression** (3) - Pre-injection compression
5. **Ready to Inject** (4) - Prepared for injection cycle
6. **Injecting** (5) - Active injection in progress

### Key Features
- **Plunger Position Control** - Visual representation and manual control of plunger tip position
- **Mould Parameter Management** - Configuration for injection parameters
- **Real-time State Monitoring** - Visual feedback of machine operation
- **Touch-based Interface** - Intuitive control via capacitive touchscreen

### Injection Parameters
The system manages the following mould parameters:

| Parameter | Default Value | Description |
|-----------|---------------|-------------|
| Mould Name | - | Identifier for parameter set |
| Fill Speed | 70 | Speed during fill phase |
| Fill Distance | 200 | Distance to travel during fill |
| Fill Acceleration | 5000 | Acceleration rate for fill |
| Hold Speed | 10 | Speed during hold/pack phase |
| Hold Distance | 5 | Distance during hold phase |
| Hold Acceleration | 3000 | Acceleration rate for hold |

## User Interface Structure
The interface consists of multiple screens accessible via navigation:

1. **Main Screen** - Primary control interface with plunger visualization
2. **Mould Settings** - Injection parameter configuration page
3. **Common Settings** - General machine settings and calibration
4. **Plunger/Tip Visualization** - Real-time animated visual representation of the injection barrel and plunger assembly that responds dynamically to position changes

### Visual Components

#### Plunger/Tip Visualization Widget
A reusable custom widget component that provides real-time visual feedback of the injection mechanism:

**Specifications:**
- **Barrel Capacity**: 250mm
- **Barrel Interior Height**: 793 pixels
- **Scale Factor**: 3.172 pixels per mm
- **Maximum Refill Bands**: 16 (with color gradient from red to blue)

**Visual Elements:**
- **Background barrel** - Outer cylinder representing the injection barrel housing (120px × 800px, white with blue shadow)
- **Barrel interior** - Inner cavity where material flows (100px × 793px, gray)
- **Plunger assembly** - Moving piston component that drives material (80px × 793px, dark gray)
- **Plunger tip** - Precision tip that controls material displacement (90px × 80px, brown)
- **Refill hole** - Material entry point visualization (90px × 96px, semi-transparent)
- **Refill bands** - Horizontal color-coded bands representing refill history, stacked from bottom (red) to top (blue)

**Functionality:**
- **Dynamic positioning** - Plunger position updates in real-time based on `plunger_tip_position` variable (range: -700 to 0)
- **Visual feedback** - Displays calculated position value as `(plunger_tip_position + 700) / 100` (normalized 0-7 scale)
- **Responsive animation** - Smooth visual response to position changes via LVGL object Y-coordinate binding
- **Multi-instance support** - Widget can be instantiated on multiple screens with synchronized state

**Interactive Controls:**
- **Test sliders** for manual plunger position control during testing and calibration
- **Machine state indicators** displaying current operational state
- **Parameter value displays** for real-time monitoring

## Data Architecture

### Global Variables
- `machine_state` (enum) - Current machine operational state
- `plunger_tip_position` (integer) - Current position of plunger tip
- `mould_parameters_list` (struct) - List of mould parameter configurations

### Structured Data Types
- **mould_parameter** structure:
  - `parameter_label` (string) - Parameter name/description
  - `parameter_value` (any) - Parameter value

### Enumerations
- **machine_state** - Six operational states
- **mould_params_options** - Configuration parameter identifiers

## Project Structure
```
injector_initial_layout/
├── include/
│   ├── lv_conf.h                          # LVGL configuration
│   └── touch.h                            # GT911 touch driver configuration
├── injector_initial_layout.eez-project    # Main EEZ Studio project file
├── injector_initial_layout.eez-project-ui-state
├── platformio.ini                         # PlatformIO environment
└── src/
    ├── main.cpp                           # Application entry point
    └── ui/                                # Generated LVGL UI code + native hooks
        ├── actions.h                      # Event handler declarations
        ├── actions_impl.cpp               # Custom action implementations
        ├── actions_impl.h                 # Custom action declarations
        ├── eez-flow.cpp                   # Flow engine implementation
        ├── eez-flow.h                     # Flow engine header
        ├── fonts.h                        # Font declarations
        ├── images.c                       # Image resources
        ├── images.h                       # Image declarations
        ├── screens.c                      # Screen implementations
        ├── screens.h                      # Screen declarations
        ├── structs.h                      # Data structure wrappers
        ├── styles.c                       # UI style implementations
        ├── styles.h                       # UI style declarations
        ├── ui.c                           # Main UI initialization
        ├── ui.h                           # UI header
        └── vars.h                         # Variable declarations
```

## Build Configuration
- **LVGL Include**: `lvgl/lvgl.h`
- **Destination Folder**: `src\ui`
- **Flow Support**: Enabled
- **Source Code Generation**: Enabled for EEZ Framework
- **Execution Queue Size**: 1000
- **Expression Evaluator Stack Size**: 20

## Development Notes
This is the **initial layout** version of the interface, designed for iterative development and testing with the ESP32-based HMI hardware. The project utilizes EEZ Studio's visual design capabilities to generate production-ready C code for the LVGL graphics library.
