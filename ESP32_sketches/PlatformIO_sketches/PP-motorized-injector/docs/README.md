# Documentation Index

**Last Updated:** January 10, 2026

This directory contains active documentation for the PP-Motorized-Injector firmware. Historical/superseded docs have been archived.

---

## 📚 Quick Navigation

### **Getting Started**
- [Phase 2 Test Plan](PHASE_2_TEST_PLAN.md) - Current hardware testing guide

### **Architecture & Design**
- [Non-Blocking Guide](NON_BLOCKING_GUIDE.md) - Non-blocking architecture patterns
- [RTR Implementation](RTR_IMPLEMENTATION_COMPLETE.md) - CAN RTR response tracking

### **Hardware Integration**
- [Display Communications Protocol](DisplayComms_Protocol.md) - UART2 display interface spec
- [Display Integration Checklist](Integration_Checklist.md) - Step-by-step display integration
- [Endstop Safety Strategy](Endstop_Safety_Strategy.md) - Collision detection logic

### **Reference**
- [Error Codes](ERROR_CODES.md) - ODrive error codes & machine errors
- [Debug Commands](DEBUGCOMMANDS_REFERENCE.md) - Serial debug command reference
- [Broadcast Quick Reference](BROADCAST_QUICK_REF.md) - ODrive CAN broadcast messages

---

## 📖 Document Summaries

### Phase 2 Test Plan
**Purpose:** Hardware testing guide for Refill, Compression, and ReadyToInject modules  
**Status:** Active - Ready for Phase 2 hardware testing  
**Audience:** Hardware testers, firmware developers

### Non-Blocking Guide
**Purpose:** Architecture patterns for non-blocking state machines  
**Status:** Reference - Core architecture document  
**Audience:** Firmware developers, code reviewers

### RTR Implementation Complete
**Purpose:** RTR response tracking implementation summary  
**Status:** Complete - Compiles successfully (Jan 10 2026)  
**Audience:** Firmware developers, testers

### Display Communications Protocol
**Purpose:** UART2 protocol specification for Antigravity display  
**Status:** Draft - Awaiting display integration (Phase 5)  
**Audience:** Display firmware developers, integration testers

### Display Integration Checklist
**Purpose:** Step-by-step guide for integrating DisplayComms module  
**Status:** Future - Phase 5 integration  
**Audience:** Firmware developers

### Endstop Safety Strategy
**Purpose:** Endstop collision detection and safety response  
**Status:** Implemented - Active safety system  
**Audience:** Hardware designers, safety reviewers

### Error Codes
**Purpose:** Comprehensive ODrive error code reference  
**Status:** Active - Updated with ERR_CAN_RTR_FAILURE (Code 10)  
**Audience:** All developers, troubleshooting

### Debug Commands Reference
**Purpose:** Serial command reference for development/testing  
**Status:** Active - Development tool  
**Audience:** Firmware developers, hardware testers

### Broadcast Quick Reference
**Purpose:** ODrive CAN broadcast message quick reference  
**Status:** Reference - CAN protocol summary  
**Audience:** Firmware developers

---

## 🗂️ Document Lifecycle

### Active Docs (Keep Updated)
- ERROR_CODES.md
- PHASE_2_TEST_PLAN.md
- RTR_IMPLEMENTATION_COMPLETE.md
- NON_BLOCKING_GUIDE.md

### Reference Docs (Stable)
- DisplayComms_Protocol.md
- Endstop_Safety_Strategy.md
- DEBUGCOMMANDS_REFERENCE.md
- BROADCAST_QUICK_REF.md

### Future Integration (Phase 5)
- Integration_Checklist.md

---

## 📝 Recently Removed (Archived)

**Date:** January 10, 2026

The following documents were removed as they are superseded by completed work:
- `ARCHITECTURE_REFACTOR_2025.md` - Historical (Phase 1 complete)
- `BROADCAST_AUDIT.md` - Superseded by BroadcastDataStore implementation
- `BROADCAST_INTEGRATION.md` - Superseded by CanBusHandlerV2 integration
- `MIGRATION_CANBUS_V2_COMPLETE.md` - Historical migration complete
- `MODULE_MOTOR_CALL_SUMMARY.md` - Historical MotorWrapper changes
- `REFACTORING_ROADMAP.md` - Outdated roadmap (Phase 1-2 complete)
- `RTR_IMPLEMENTATION_PLAN.md` - Superseded by RTR_IMPLEMENTATION_COMPLETE.md

---

## 🔄 Version History

| Date | Change | Docs Affected |
|------|--------|---------------|
| Jan 10, 2026 | RTR implementation complete | +RTR_IMPLEMENTATION_COMPLETE.md, ERROR_CODES.md |
| Jan 10, 2026 | Archived 7 outdated docs | -7 historical docs |
| Jan 8, 2026 | Phase 2 modules complete | PHASE_2_TEST_PLAN.md |
| Jan 7, 2026 | Non-blocking refactor | NON_BLOCKING_GUIDE.md |

---

## 📞 Contact & Contributions

For documentation updates or questions:
- See [.github/copilot-instructions.md](../.github/copilot-instructions.md) for project status
- Documentation should follow Markdown best practices
- Keep summaries concise and actionable
