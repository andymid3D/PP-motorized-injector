# 📥 DoW Inbox

## 📅 2026-01-08

### 📝 Brain Dump #1: Injection Machine Display Project
- **Topic**: ESP32 Display + EEZ Studio (LVGL) + ESP32 Controller.
- **Key Details**:
    - Project: Motorized Plastic Injection Machine.
    - Hardware: Elecrow 5" ESP32 Screen (Display) + ESP32 Controller.
    - Tech: EEZ Studio 0.24, LVGL, `SafeString` library for UART.
    - Communication: Mapping UART for "Mould Profiles" (Params).
    - **Display Pins**: Tx GPIO43, Rx GPIO44.
    - **Controller Pins**: Serial 2 (Tx 17, Rx 16).
    - **Baudrate**: 115200.
    - **Logic**: Using EEZ Flow for variable mapping.
- **Workspace Preference**: Move to `~/Documents/Github` for version control.
- **Director's Note**: High Utility. Technical mapping confirmed.

---

## 🗑️ Archive / Low Priority
*(Nothing yet)*

- **Workspace Cleanup**: Redundant ESP32-Display folder removed. Files consolidated in .
- **Artifact Location**: Implementation Plan and Task files are system artifacts stored in the Antigravity 'brain' directory, but accessible via the UI links.