const elements = {
  serviceStatus: document.querySelector("#service-status"),
  serviceLabel: document.querySelector("#service-label"),
  updatedAt: document.querySelector("#updated-at"),
  sourcePath: document.querySelector("#source-path"),
  windowStatus: document.querySelector("#window-status"),
  cameraStatus: document.querySelector("#camera-status"),
  mapStatus: document.querySelector("#map-status"),
  statePath: document.querySelector("#state-path"),
  windowTitle: document.querySelector("#window-title"),
  windowImage: document.querySelector("#window-image"),
  windowEmpty: document.querySelector("#window-empty"),
  debugImage: document.querySelector("#debug-image"),
  debugEmpty: document.querySelector("#debug-empty"),
  debugAge: document.querySelector("#debug-age"),
  mapSelect: document.querySelector("#map-select"),
  mapGrid: document.querySelector("#map-grid"),
  mapMode: document.querySelector("#map-mode"),
  wallCount: document.querySelector("#wall-count"),
  boxCount: document.querySelector("#box-count"),
  goalCount: document.querySelector("#goal-count"),
  completedCount: document.querySelector("#completed-count"),
  cameraSettings: document.querySelector("#camera-settings"),
  warningCount: document.querySelector("#warning-count"),
  warningList: document.querySelector("#warning-list"),
};

let latestState = null;
let selectedMapName = null;

function setMedia(image, empty, available, url, token) {
  if (!available) {
    image.classList.remove("visible");
    empty.classList.remove("hidden");
    image.removeAttribute("src");
    delete image.dataset.artifactToken;
    return;
  }
  const artifactToken = String(token);
  if (image.dataset.artifactToken === artifactToken && image.getAttribute("src")) {
    return;
  }
  image.dataset.artifactToken = artifactToken;
  image.onload = () => {
    image.classList.add("visible");
    empty.classList.add("hidden");
  };
  image.onerror = () => {
    image.classList.remove("visible");
    empty.classList.remove("hidden");
  };
  image.src = `${url}?v=${encodeURIComponent(artifactToken)}`;
}

function renderMap(map) {
  elements.mapGrid.replaceChildren();
  if (!map) {
    elements.mapMode.textContent = "--";
    return;
  }

  const classes = { "#": "wall", "$": "box", ".": "goal", "*": "completed" };
  const fragment = document.createDocumentFragment();
  map.grid.forEach((row) => {
    [...row].forEach((cell) => {
      const node = document.createElement("span");
      node.className = `map-cell ${classes[cell] || "floor"}`;
      node.title = cell;
      fragment.appendChild(node);
    });
  });
  elements.mapGrid.appendChild(fragment);
  elements.mapMode.textContent = map.mode === "pure_sokoban" ? "纯推箱" : "图像分类";
  elements.wallCount.textContent = map.counts.walls;
  elements.boxCount.textContent = map.counts.boxes;
  elements.goalCount.textContent = map.counts.goals;
  elements.completedCount.textContent = map.counts.completed;
}

function renderMapSelector(maps, candidate) {
  const names = maps.map((map) => map.name);
  if (!selectedMapName || !names.includes(selectedMapName)) {
    selectedMapName = candidate || names[0] || null;
  }
  elements.mapSelect.replaceChildren();
  maps.forEach((map) => {
    const option = document.createElement("option");
    option.value = map.name;
    option.textContent = map.name;
    option.selected = map.name === selectedMapName;
    elements.mapSelect.appendChild(option);
  });
  elements.mapSelect.disabled = maps.length === 0;
  renderMap(maps.find((map) => map.name === selectedMapName));
}

function renderSettings(settings) {
  const labels = {
    camera_index: "camera_index",
    compress_quality: "compress_quality",
    h_range: "h_range",
    s_range: "s_range",
    v_range: "v_range",
  };
  elements.cameraSettings.replaceChildren();
  Object.entries(labels).forEach(([key, label]) => {
    const row = document.createElement("div");
    const term = document.createElement("dt");
    const value = document.createElement("dd");
    term.textContent = label;
    value.textContent = settings[key] ?? "--";
    row.append(term, value);
    elements.cameraSettings.appendChild(row);
  });
}

function renderWarnings(warnings) {
  elements.warningCount.textContent = warnings.length;
  elements.warningList.replaceChildren();
  if (warnings.length === 0) {
    const item = document.createElement("li");
    item.className = "clear";
    item.textContent = "当前采集源无异常";
    elements.warningList.appendChild(item);
    return;
  }
  warnings.forEach((warning) => {
    const item = document.createElement("li");
    item.textContent = warning;
    elements.warningList.appendChild(item);
  });
}

function render(state) {
  latestState = state;
  const generated = new Date(state.generated_at);
  elements.updatedAt.textContent = generated.toLocaleTimeString("zh-CN", { hour12: false });
  elements.sourcePath.textContent = state.smartcar_dir;
  elements.statePath.textContent = state.artifacts.state_json;
  elements.serviceStatus.className = "status-dot status-online";
  elements.serviceLabel.textContent = "采集运行中";

  const app = state.app;
  elements.windowStatus.textContent = app.window_found ? "已连接" : "等待窗口";
  elements.windowTitle.textContent = app.window_title || "等待窗口";
  setMedia(
    elements.windowImage,
    elements.windowEmpty,
    app.capture_updated,
    "/api/window",
    state.generated_at,
  );

  const debug = state.camera.debug_image;
  elements.cameraStatus.textContent = debug.exists ? `设备 ${state.camera.settings.camera_index ?? "--"}` : "等待图像";
  if (debug.exists) {
    const ageSeconds = Math.max(0, Math.round(Date.now() / 1000 - debug.modified_at));
    elements.debugAge.textContent = `${ageSeconds}s 前更新`;
  } else {
    elements.debugAge.textContent = "无图像";
  }
  setMedia(elements.debugImage, elements.debugEmpty, debug.exists, "/api/debug", debug.modified_at);

  elements.mapStatus.textContent = state.selected_map_candidate || "无地图";
  renderMapSelector(state.maps, state.selected_map_candidate);
  renderSettings(state.camera.settings);
  renderWarnings(state.warnings);
}

async function refresh() {
  try {
    const response = await fetch(`/api/state?t=${Date.now()}`, { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    render(await response.json());
  } catch (error) {
    elements.serviceStatus.className = "status-dot status-error";
    elements.serviceLabel.textContent = "采集连接中断";
  }
}

elements.mapSelect.addEventListener("change", (event) => {
  selectedMapName = event.target.value;
  if (latestState) {
    renderMap(latestState.maps.find((map) => map.name === selectedMapName));
  }
});

refresh();
setInterval(refresh, 1000);
