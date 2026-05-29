# Task Execution Plan: Joystick Reconnection Failure Analysis and Resolution Design

## 1. Requirement Analysis
- **Core Goal**: Analyze why the Bluetooth joystick fails to reconnect after disconnection in the current codebase, explain the root cause, and design a robust solution to fix the issue.
- **Scope Explanation**: Includes structural code analysis of `src/hal/hal_joystick.cpp`, `src/hal/hal_joystick.h`, and `src/main.cpp`, identification of race conditions and state management bugs, and providing concrete code changes to solve the issue. Does not include hardware troubleshooting of physical Bluetooth signal range.

## 2. Task Checklist
- [x] **Phase 1: Code Research**
    - [x] Analyze `hal_joystick.cpp` and `hal_joystick.h` to understand the connection, disconnection, and update loops.
    - [x] Identify the relationship between Bluepad32 background thread callbacks and the main `Gamepad_Task` thread.
- [x] **Phase 2: Root Cause Analysis**
    - [x] Document the primary bugs in state tracking, thread safety, and stale pointer references.
    - [x] Trace step-by-step why reconnection fails.
- [x] **Phase 3: Solution Design & Verification Plan**
    - [x] Design a thread-safe, robust connection state tracking logic.
    - [x] Formulate exact code changes for `hal_joystick.h` and `hal_joystick.cpp`.
    - [x] Detail verification steps to confirm the fix.

## 3. Risks and Countermeasures
- **Data Races between RTOS Tasks** -> Use appropriate mutex or simple atomic checks if applicable, or design state transitions such that updates only happen from a single thread context.
- **Null Pointer Dereference on Disconnect** -> Explicitly set pointers to `nullptr` as soon as they are detected as disconnected.
