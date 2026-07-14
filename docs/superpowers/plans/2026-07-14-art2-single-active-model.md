# ART2 Single Active Model Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the formal OpenART2 UART script use the same one-model classification path as the proven standalone test script.

**Architecture:** Keep one active framebuffer model. A `VISION_MODE` command requests a mode; the main loop releases the previous model, loads the requested model and labels, then sends `VISION_READY`. Classification and UART sample formatting remain unchanged.

**Tech Stack:** OpenMV MicroPython, `tf`, UART4 text protocol, Python AST host tests.

---

### Task 1: Lock Mode Loading Semantics

**Files:**
- Modify: `tests/art2_protocol_test.py`

- [ ] Add tests proving a new mode command does not reply READY before loading.
- [ ] Add tests proving a loaded same-mode retry replies READY without reloading.
- [ ] Add tests proving BOX to TARGET releases the previous model and loads only the target model.
- [ ] Run `python tests/art2_protocol_test.py` and confirm it fails against the current two-model implementation.

### Task 2: Implement One Active Model

**Files:**
- Modify: `openmv/视觉/main.py`

- [ ] Replace the two startup model objects with `pending_mode`, `active_net`, and `active_labels`.
- [ ] Handle `VISION_MODE` as a load request; send READY only after successful activation.
- [ ] Release the previous framebuffer model before loading the requested model.
- [ ] Preserve the standalone camera configuration and `tf.classify(...)[0].output()` path.
- [ ] On load failure, send no READY and wait for the MCU mode retry.
- [ ] Run the ART2 protocol and Python syntax tests.

### Task 3: Deploy and Regress

**Files:**
- Deploy: `F:/main.py`

- [ ] Copy the verified formal script to the SD root and compare SHA-256.
- [ ] Run UART4, subject2 logic, and subject2 scan host tests.
- [ ] Run `git diff --check`.
