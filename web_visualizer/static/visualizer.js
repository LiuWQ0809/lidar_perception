const statusText = document.getElementById('status-text');
const statusDot = document.getElementById('status-dot');
const cameraSelect = document.getElementById('camera-select');
const mjpeg = document.getElementById('mjpeg');
const imageFps = document.getElementById('image-fps');
const imageAge = document.getElementById('image-age');
const markerCount = document.getElementById('marker-count');
const markerFps = document.getElementById('marker-fps');
const markerAge = document.getElementById('marker-age');
const objectList = document.getElementById('object-list');
const listMeta = document.getElementById('list-meta');
const mapMeta = document.getElementById('map-meta');
const mapCanvas = document.getElementById('map-canvas');
const fitMap = document.getElementById('fit-map');
const toggleGrid = document.getElementById('toggle-grid');
const toggleSort = document.getElementById('toggle-sort');

let currentCamera = null;
let markers = [];
let mapRange = { x_min: -5, x_max: 20, y_min: -5, y_max: 5 };
let showGrid = true;
let sortMode = 'distance';

function setStatus(ok, text) {
  statusText.textContent = text;
  if (ok) {
    statusDot.style.background = '#3ddc97';
    statusDot.style.boxShadow = '0 0 12px rgba(61, 220, 151, 0.8)';
  } else {
    statusDot.style.background = '#f5b342';
    statusDot.style.boxShadow = '0 0 12px rgba(245, 179, 66, 0.8)';
  }
}

function formatAge(seconds) {
  if (seconds === null || seconds === undefined) {
    return '--';
  }
  if (seconds < 0) {
    return '--';
  }
  return `${seconds.toFixed(2)}s`;
}

function updateStream() {
  const url = currentCamera ? `/stream/${encodeURIComponent(currentCamera)}` : '/stream';
  mjpeg.src = url;
}

async function refreshStatus() {
  try {
    const cameraParam = currentCamera ? `?camera=${encodeURIComponent(currentCamera)}` : '';
    const res = await fetch(`/api/status${cameraParam}`);
    if (!res.ok) throw new Error('status');
    const data = await res.json();
    setStatus(true, 'online');

    if (Array.isArray(data.cameras)) {
      renderCameraSelect(data.cameras, data.default_camera);
    }

    imageFps.textContent = data.image.fps.toFixed(1);
    imageAge.textContent = formatAge(data.image.age);
    markerCount.textContent = data.markers.count;
    markerFps.textContent = data.markers.fps.toFixed(1);
    markerAge.textContent = formatAge(data.markers.age);

    if (data.image.camera) {
      currentCamera = data.image.camera;
    }

    const meta = data.image.width
      ? `${data.image.width}x${data.image.height} · ${data.image.camera || '--'}`
      : `camera: ${data.image.camera || '--'}`;
    document.getElementById('camera-meta').textContent = meta;
  } catch (err) {
    setStatus(false, 'offline');
  }
}

async function refreshMarkers() {
  try {
    const res = await fetch('/api/markers');
    if (!res.ok) throw new Error('markers');
    const data = await res.json();
    markers = data.markers || [];
    if (data.range) {
      mapRange = data.range;
    }
    listMeta.textContent = `${markers.length} objects`;
    mapMeta.textContent = `x: ${mapRange.x_min}~${mapRange.x_max}m, y: ${mapRange.y_min}~${mapRange.y_max}m`;
    renderList();
    drawMap();
  } catch (err) {
    // ignore
  }
}

function renderCameraSelect(cameras, fallback) {
  if (!cameras.length) return;
  const previous = currentCamera || fallback || cameras[0];
  if (!cameraSelect.options.length) {
    cameras.forEach((cam) => {
      const opt = document.createElement('option');
      opt.value = cam;
      opt.textContent = cam;
      cameraSelect.appendChild(opt);
    });
  } else if (cameraSelect.options.length !== cameras.length) {
    cameraSelect.innerHTML = '';
    cameras.forEach((cam) => {
      const opt = document.createElement('option');
      opt.value = cam;
      opt.textContent = cam;
      cameraSelect.appendChild(opt);
    });
  }
  cameraSelect.value = previous;
  currentCamera = previous;
  updateStream();
}

function renderList() {
  const list = [...markers];
  if (sortMode === 'distance') {
    list.sort((a, b) => a.distance - b.distance);
  } else if (sortMode === 'class') {
    list.sort((a, b) => a.class_hint.localeCompare(b.class_hint));
  }

  objectList.innerHTML = '';
  if (!list.length) {
    objectList.innerHTML = '<div class="object-item">No detections</div>';
    return;
  }

  list.forEach((obj) => {
    const item = document.createElement('div');
    item.className = 'object-item';
    const badge = document.createElement('span');
    badge.className = `badge ${obj.class_hint}`;
    badge.textContent = obj.class_hint;
    const meta = document.createElement('div');
    meta.className = 'meta';
    meta.innerHTML = `<span>#${obj.id}</span><span>${obj.size_x.toFixed(1)}×${obj.size_y.toFixed(1)}×${obj.size_z.toFixed(1)}</span>`;
    const left = document.createElement('div');
    left.className = 'meta';
    left.appendChild(badge);
    left.appendChild(meta);
    const distance = document.createElement('div');
    distance.className = 'distance';
    distance.textContent = `${obj.distance.toFixed(1)}m`;
    item.appendChild(left);
    item.appendChild(distance);
    objectList.appendChild(item);
  });
}

function drawMap() {
  const ctx = mapCanvas.getContext('2d');
  const w = mapCanvas.width;
  const h = mapCanvas.height;
  ctx.clearRect(0, 0, w, h);

  const pad = 40;
  const xMin = mapRange.x_min;
  const xMax = mapRange.x_max;
  const yMin = mapRange.y_min;
  const yMax = mapRange.y_max;
  const xSpan = xMax - xMin;
  const ySpan = yMax - yMin;

  const scale = Math.min((w - 2 * pad) / xSpan, (h - 2 * pad) / ySpan);
  const originX = pad + (-xMin) * scale;
  const originY = h - pad - (-yMin) * scale;

  ctx.fillStyle = '#0b0f17';
  ctx.fillRect(0, 0, w, h);

  if (showGrid) {
    ctx.strokeStyle = 'rgba(255,255,255,0.06)';
    ctx.lineWidth = 1;
    const step = 1;
    for (let x = Math.ceil(xMin); x <= xMax; x += step) {
      const px = originX + x * scale;
      ctx.beginPath();
      ctx.moveTo(px, pad);
      ctx.lineTo(px, h - pad);
      ctx.stroke();
    }
    for (let y = Math.ceil(yMin); y <= yMax; y += step) {
      const py = originY - y * scale;
      ctx.beginPath();
      ctx.moveTo(pad, py);
      ctx.lineTo(w - pad, py);
      ctx.stroke();
    }
  }

  ctx.strokeStyle = 'rgba(255,255,255,0.18)';
  ctx.lineWidth = 1.5;
  ctx.strokeRect(pad, pad, w - 2 * pad, h - 2 * pad);

  markers.forEach((obj) => {
    const x = originX + obj.x * scale;
    const y = originY - obj.y * scale;
    const width = obj.size_x * scale;
    const height = obj.size_y * scale;
    const yaw = obj.yaw;
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(-yaw);
    ctx.strokeStyle = colorFromClass(obj.class_hint, 0.9);
    ctx.fillStyle = colorFromClass(obj.class_hint, 0.2);
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.rect(-width / 2, -height / 2, width, height);
    ctx.fill();
    ctx.stroke();
    ctx.restore();
  });

  ctx.fillStyle = '#f5b342';
  ctx.beginPath();
  ctx.arc(originX, originY, 6, 0, Math.PI * 2);
  ctx.fill();
}

function colorFromClass(cls, alpha) {
  let base = [77, 163, 255];
  if (cls === 'person') base = [255, 81, 81];
  if (cls === 'car') base = [61, 220, 151];
  return `rgba(${base[0]}, ${base[1]}, ${base[2]}, ${alpha})`;
}

cameraSelect.addEventListener('change', (event) => {
  currentCamera = event.target.value;
  updateStream();
});

fitMap.addEventListener('click', () => drawMap());

toggleGrid.addEventListener('click', () => {
  showGrid = !showGrid;
  toggleGrid.textContent = showGrid ? 'Grid' : 'Grid off';
  drawMap();
});

toggleSort.addEventListener('click', () => {
  sortMode = sortMode === 'distance' ? 'class' : 'distance';
  toggleSort.textContent = `Sort: ${sortMode}`;
  renderList();
});

updateStream();
refreshStatus();
refreshMarkers();
setInterval(refreshStatus, 1500);
setInterval(refreshMarkers, 500);

window.addEventListener('resize', () => {
  const rect = mapCanvas.getBoundingClientRect();
  const size = Math.min(rect.width, 600);
  mapCanvas.width = size;
  mapCanvas.height = size;
  drawMap();
});

// init canvas size
const rect = mapCanvas.getBoundingClientRect();
const size = Math.min(rect.width, 600);
mapCanvas.width = size;
mapCanvas.height = size;
