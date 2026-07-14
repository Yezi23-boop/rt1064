# SmartCar Agent Observer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a read-only local observer that turns SmartCar VR maps, camera settings, debug imagery, and the live program window into stable JSON and image endpoints that an Agent can inspect while tuning.

**Architecture:** A small Python service polls the vendor directory, parses map/config files, captures the SmartCar window through Win32 plus Pillow, and atomically publishes `runtime/latest_state.json` and `runtime/latest_window.png`. A dependency-free local web panel consumes the same `/api/state`, `/api/window`, and `/api/debug` endpoints, so the browser and Agent share one source of truth.

**Tech Stack:** Python 3 standard library, Pillow, Win32 `ctypes`, HTML/CSS/JavaScript, `unittest`.

---

### Task 1: Define Offline Parsing Contracts

**Files:**
- Create: `tools/smartcar_agent/tests/test_observer.py`
- Create: `tools/smartcar_agent/observer.py`

- [ ] Add a failing test for parsing a 16x12 map and counting walls, boxes, goals, and completed boxes.
- [ ] Add a failing test that rejects maps with the wrong dimensions.
- [ ] Add a failing test for `camera.ini` integer values.
- [ ] Implement only the parsers needed to pass those tests.

### Task 2: Publish an Agent Snapshot

**Files:**
- Modify: `tools/smartcar_agent/tests/test_observer.py`
- Modify: `tools/smartcar_agent/observer.py`

- [ ] Add a failing test for a snapshot built from a temporary vendor directory.
- [ ] Include camera configuration, all valid maps, latest-map candidate, debug-image metadata, window status, warnings, and stable absolute artifact paths.
- [ ] Write JSON atomically so the Agent never reads a partial file.
- [ ] Keep missing program/window/debug image states non-fatal and visible in `warnings`.

### Task 3: Capture the SmartCar Window

**Files:**
- Modify: `tools/smartcar_agent/tests/test_observer.py`
- Modify: `tools/smartcar_agent/observer.py`

- [ ] Add a failing test for deterministic window-title selection from injected candidates.
- [ ] Enumerate visible top-level Win32 windows and prefer titles containing `SmartCar_VR` or the Chinese product name.
- [ ] Capture the selected window bounding box with Pillow and publish `latest_window.png`.
- [ ] Report minimized, absent, or off-screen windows without terminating the observer.

### Task 4: Serve the Local Dashboard

**Files:**
- Create: `tools/smartcar_agent/server.py`
- Create: `tools/smartcar_agent/web/index.html`
- Create: `tools/smartcar_agent/web/app.js`
- Create: `tools/smartcar_agent/web/styles.css`
- Create: `tools/smartcar_agent/run.ps1`

- [ ] Poll at a short fixed interval and refresh the snapshot and optional window capture.
- [ ] Serve `/api/state`, `/api/window`, and `/api/debug` on localhost only.
- [ ] Build a dense engineering panel for connection status, camera parameters, map grid, warnings, debug image, and game window.
- [ ] Start the service with the known vendor directory by default and allow one CLI override.

### Task 5: Document and Verify

**Files:**
- Create: `tools/smartcar_agent/README.md`
- Modify: `.gitignore`
- Modify: `docs/agents/file-map.md`

- [ ] Document startup, Agent-readable paths, current limitations, and why ports 8888/8889 are not opened by this version.
- [ ] Ignore generated runtime artifacts.
- [ ] Run focused unit tests and `git diff --check`.
- [ ] Start the service, exercise every HTTP endpoint, and visually verify the desktop dashboard in the in-app browser.

---

## Self-Review

- Scope is confined to a new host-side tool plus navigation/ignore documentation.
- The official EXE, camera helper, OpenART scripts, UART protocols, and RT1064 control code remain unchanged.
- The first version observes existing artifacts and the visible window; exact internal Godot state and the occupied camera TCP streams are deliberately out of scope.
