# VOFA Output Mode Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Simplify VOFA into a single-direction debug CSV output module with unified output modes and no joystick receive/control path.

**Architecture:** `vofa.h` defines one output mode macro and send period. `vofa.c` keeps only `vofa_init()` and `vofa_service()` plus small send helpers for control, pose, and ART debug CSV. ART/executor debug data is exposed through lightweight getters instead of letting VOFA depend on internal static variables.

**Tech Stack:** RT1064 C firmware, existing wireless `printf`, VOFA+ FireWater CSV, Keil MDK.

---

### Task 1: Replace VOFA Boolean Output Switches

**Files:**
- Modify: `project/user/inc/vofa.h`

- [ ] Define `vofa_output_mode_enum` with `OFF`, `CONTROL`, `POSE`, `ART`.
- [ ] Replace `VOFA_CURVE_OUTPUT_ENABLE` and `VOFA_POSE_ONLY_ENABLE` with `VOFA_OUTPUT_MODE`.
- [ ] Delete joystick-related macros: `VOFA_JOYSTICK_CONTROL_ENABLE`, timeout, max abs, deadband, RX chunk, RX line.

### Task 2: Add Debug Getters

**Files:**
- Modify: `project/user/inc/executor.h`
- Modify: `project/user/src/executor.c`
- Modify: `project/user/inc/art_replan.h`
- Modify: `project/user/src/art_replan.c`

- [ ] Add `executor_debug_status_struct` containing current target, error, step, action, state, ART center result and center correction deltas.
- [ ] Add `executor_get_debug_status(executor_debug_status_struct *status)`.
- [ ] Add `art_replan_debug_status_struct` containing phase, stable count, confirmed B/T count and ART wait state.
- [ ] Add `art_replan_get_debug_status(art_replan_debug_status_struct *status)`.

### Task 3: Simplify `vofa.c`

**Files:**
- Modify: `project/user/src/vofa.c`

- [ ] Delete joystick receive/parser/apply code.
- [ ] Keep `vofa_last_send_ms`.
- [ ] Split output into `vofa_send_control()`, `vofa_send_pose()`, `vofa_send_art()`.
- [ ] Use `VOFA_OUTPUT_MODE` in `vofa_service()`.

### Task 4: Verify

**Files:**
- Verify only.

- [ ] Static check that joystick symbols are gone from `vofa.c/.h`.
- [ ] Static check that `VOFA_OUTPUT_MODE` controls output.
- [ ] Keil compile with `0 Error(s), 0 Warning(s)`.
- [ ] Field check ART mode CSV order:
  `pose_x,pose_y,yaw,target_x,target_y,err_x,err_y,art_result,art_dx,art_dy,art_diff,step,total,action,state,art_frame,art_age_ms`

---

## Self-Review

- Scope is focused on VOFA debug output and small debug getters.
- It does not change control PID, ART UART protocol, solver, or OpenART recognition.
- Joystick receive is explicitly removed because the user no longer needs it.
