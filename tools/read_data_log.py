#!/usr/bin/env python3
"""
read_data_log.py — Web viewer for HackyRacingRobot JSONL data logs.

Opens log files from ~/Documents/HackyRacingRobot/ (or --dir) and serves an
interactive browser dashboard on port 5004.

Features
--------
  File browser   : list all .jsonl files with size and duration
  Drive chart    : left / right motor commands over time
  GPS track      : Leaflet map overlay (skipped when no GPS fix)
  Telemetry      : voltage, heading, temps over time
  LiDAR          : polar plot for any selected frame
  Mode timeline  : colour-coded mode bands across the run
  Frame inspector: raw JSON for any frame

Usage
-----
  python3 tools/read_data_log.py
  python3 tools/read_data_log.py --dir /path/to/logs --port 5004

Dependencies
------------
  pip install flask
  (Chart.js and Leaflet loaded from CDN)
"""

import argparse
import json
import math
import os
import sys
from pathlib import Path

from flask import Flask, Response, jsonify, request

app = Flask(__name__)

_log_dir = str(Path.home() / 'Documents' / 'HackyRacingRobot')


# ── File helpers ───────────────────────────────────────────────────────────────

def _list_files():
    """Return list of dicts for every .jsonl file in _log_dir."""
    out = []
    try:
        for p in sorted(Path(_log_dir).glob('*.jsonl'), reverse=True):
            size = p.stat().st_size
            out.append({'name': p.name, 'path': str(p), 'size': size,
                        'size_kb': round(size / 1024, 1)})
    except FileNotFoundError:
        pass
    return out


def _load_file(path: str):
    """Parse a JSONL file; return list of record dicts."""
    records = []
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    records.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
    return records


def _strip_lidar(records):
    """Return records with lidar angles/distances removed (reduces payload size)."""
    stripped = []
    for r in records:
        s = dict(r)
        if 'lidar' in s and s['lidar']:
            s['lidar'] = {'ts': s['lidar'].get('ts')}
        stripped.append(s)
    return stripped


def _file_summary(records):
    """Return a summary dict from a list of records."""
    if not records:
        return {}
    t0 = records[0].get('ts', 0)
    t1 = records[-1].get('ts', 0)
    duration = round(t1 - t0, 1)

    mode_counts = {}
    gps_ok = 0
    for r in records:
        m = r.get('mode', 'UNKNOWN')
        mode_counts[m] = mode_counts.get(m, 0) + 1
        if r.get('gps', {}).get('fix', 0) >= 1:
            g = r['gps']
            if g.get('lat') and g.get('lon'):
                gps_ok += 1

    return {
        'count':      len(records),
        'duration_s': duration,
        'mode_counts': mode_counts,
        'gps_frames': gps_ok,
        'hz':         round(len(records) / duration, 1) if duration > 0 else 0,
    }


# ── API routes ─────────────────────────────────────────────────────────────────

@app.get('/api/files')
def api_files():
    return jsonify(_list_files())


@app.get('/api/load')
def api_load():
    """Load a JSONL file; strip LiDAR bulk data for fast charting."""
    path = request.args.get('path', '')
    if not path or not os.path.isfile(path):
        return jsonify({'error': 'File not found'}), 404
    try:
        records  = _load_file(path)
        stripped = _strip_lidar(records)
        summary  = _file_summary(records)
        return jsonify({'summary': summary, 'records': stripped})
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.get('/api/frame')
def api_frame():
    """Return a single full record (including LiDAR) by index."""
    path = request.args.get('path', '')
    idx  = request.args.get('idx', type=int, default=0)
    if not path or not os.path.isfile(path):
        return jsonify({'error': 'File not found'}), 404
    try:
        records = _load_file(path)
        if idx < 0 or idx >= len(records):
            return jsonify({'error': 'Index out of range'}), 400
        return jsonify(records[idx])
    except Exception as e:
        return jsonify({'error': str(e)}), 500


# ── Main page ──────────────────────────────────────────────────────────────────

@app.get('/')
def index():
    return Response(_HTML, mimetype='text/html')


# ── HTML / JS frontend ─────────────────────────────────────────────────────────

_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>HackyRacingRobot — Data Log Viewer</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4/dist/chart.umd.min.js"></script>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{font-family:monospace;background:#111;color:#ccc;display:flex;height:100vh;overflow:hidden}

  /* ── sidebar ── */
  #sidebar{width:220px;min-width:180px;background:#1a1a1a;border-right:1px solid #333;
           display:flex;flex-direction:column;overflow:hidden}
  #sidebar h2{padding:10px 12px;font-size:13px;color:#8af;border-bottom:1px solid #333;
              letter-spacing:1px;text-transform:uppercase}
  #file-list{flex:1;overflow-y:auto;padding:6px 0}
  .file-entry{padding:8px 12px;cursor:pointer;font-size:11px;border-bottom:1px solid #222;
              transition:background .15s}
  .file-entry:hover{background:#2a2a2a}
  .file-entry.active{background:#1e3a5f;color:#8af}
  .file-name{color:#eee;word-break:break-all}
  .file-meta{color:#666;font-size:10px;margin-top:2px}
  #no-files{padding:16px 12px;color:#555;font-size:11px}

  /* ── main area ── */
  #main{flex:1;display:flex;flex-direction:column;overflow:hidden}

  /* summary bar */
  #summary{background:#1a1a1a;border-bottom:1px solid #333;padding:8px 14px;
           display:flex;flex-wrap:wrap;gap:12px;font-size:11px;align-items:center;min-height:36px}
  .badge{background:#222;border:1px solid #444;border-radius:4px;padding:3px 8px;color:#aaa}
  .badge span{color:#fff;font-weight:bold}
  #summary-empty{color:#555;font-size:11px}

  /* tabs */
  #tabs{display:flex;background:#181818;border-bottom:1px solid #333}
  .tab{padding:8px 16px;cursor:pointer;font-size:12px;color:#777;border-bottom:2px solid transparent;
       transition:.15s;user-select:none}
  .tab:hover{color:#aaa}
  .tab.active{color:#8af;border-bottom-color:#8af}

  /* panels */
  #panels{flex:1;overflow:hidden;position:relative}
  .panel{display:none;height:100%;overflow:auto;padding:10px}
  .panel.active{display:flex;flex-direction:column;gap:10px}

  /* chart cards */
  .card{background:#1a1a1a;border:1px solid #333;border-radius:6px;padding:12px}
  .card h3{font-size:11px;color:#888;letter-spacing:1px;text-transform:uppercase;margin-bottom:8px}
  .chart-wrap{position:relative;height:160px}

  /* GPS map */
  #map{height:380px;border-radius:6px;border:1px solid #333}

  /* LiDAR */
  #lidar-canvas{background:#0d0d0d;border-radius:50%;border:1px solid #333;display:block;margin:0 auto}
  #lidar-controls{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
  #frame-slider{flex:1;min-width:120px;accent-color:#8af}
  #frame-label{font-size:11px;color:#888;min-width:80px}
  #frame-ts{font-size:11px;color:#666}

  /* frame inspector */
  #frame-json{background:#0d0d0d;border:1px solid #333;border-radius:6px;
              padding:12px;font-size:11px;white-space:pre-wrap;color:#aaa;
              max-height:calc(100vh - 200px);overflow-y:auto;flex:1}

  /* mode timeline */
  #mode-canvas{width:100%;height:36px;display:block;border-radius:4px;cursor:crosshair}

  /* loading overlay */
  #loading{position:fixed;inset:0;background:rgba(0,0,0,.7);display:none;
           align-items:center;justify-content:center;z-index:999;font-size:16px;color:#8af}
</style>
</head>
<body>

<div id="sidebar">
  <h2>Data Logs</h2>
  <div id="file-list"><div id="no-files">Loading…</div></div>
</div>

<div id="main">
  <div id="summary"><span id="summary-empty">Select a log file</span></div>

  <div id="tabs">
    <div class="tab active" data-tab="drive">Drive</div>
    <div class="tab" data-tab="telemetry">Telemetry</div>
    <div class="tab" data-tab="gps">GPS</div>
    <div class="tab" data-tab="lidar">LiDAR</div>
    <div class="tab" data-tab="inspector">Inspector</div>
  </div>

  <div id="panels">

    <!-- Drive panel -->
    <div class="panel active" id="panel-drive">
      <div class="card">
        <h3>Mode Timeline</h3>
        <canvas id="mode-canvas"></canvas>
      </div>
      <div class="card">
        <h3>Motor Commands (left / right)</h3>
        <div class="chart-wrap"><canvas id="chart-drive"></canvas></div>
      </div>
      <div class="card">
        <h3>RC Channels — throttle (CH3) &amp; steer (CH1)</h3>
        <div class="chart-wrap"><canvas id="chart-rc"></canvas></div>
      </div>
    </div>

    <!-- Telemetry panel -->
    <div class="panel" id="panel-telemetry">
      <div class="card">
        <h3>Voltage (V)</h3>
        <div class="chart-wrap"><canvas id="chart-voltage"></canvas></div>
      </div>
      <div class="card">
        <h3>IMU Heading (°)</h3>
        <div class="chart-wrap"><canvas id="chart-heading"></canvas></div>
      </div>
      <div class="card">
        <h3>Board / Motor Temps (°C)</h3>
        <div class="chart-wrap"><canvas id="chart-temp"></canvas></div>
      </div>
      <div class="card">
        <h3>CPU % / Temp</h3>
        <div class="chart-wrap"><canvas id="chart-cpu"></canvas></div>
      </div>
    </div>

    <!-- GPS panel -->
    <div class="panel" id="panel-gps">
      <div id="map"></div>
      <div class="card">
        <h3>GPS Speed (m/s)</h3>
        <div class="chart-wrap"><canvas id="chart-speed"></canvas></div>
      </div>
    </div>

    <!-- LiDAR panel -->
    <div class="panel" id="panel-lidar">
      <div class="card">
        <h3>LiDAR Polar Plot</h3>
        <div id="lidar-controls">
          <input type="range" id="frame-slider" min="0" value="0">
          <span id="frame-label">Frame 0</span>
          <span id="frame-ts"></span>
        </div>
        <canvas id="lidar-canvas" width="480" height="480"></canvas>
      </div>
    </div>

    <!-- Inspector panel -->
    <div class="panel" id="panel-inspector">
      <div class="card" style="flex-shrink:0">
        <h3>Frame Inspector</h3>
        <div id="insp-controls" style="display:flex;align-items:center;gap:10px;flex-wrap:wrap;margin-bottom:8px">
          <input type="range" id="insp-slider" min="0" value="0" style="flex:1;min-width:120px;accent-color:#8af">
          <span id="insp-label" style="font-size:11px;color:#888;min-width:80px">Frame 0</span>
        </div>
        <pre id="frame-json">Select a file to inspect frames</pre>
      </div>
    </div>

  </div>
</div>

<div id="loading">Loading…</div>

<script>
// ── state ──────────────────────────────────────────────────────────────────────
let _records = [];
let _currentPath = '';
let _charts = {};
let _map = null;
let _gpsLayer = null;
let _robotMarker = null;

// ── tabs ───────────────────────────────────────────────────────────────────────
document.querySelectorAll('.tab').forEach(t => {
  t.addEventListener('click', () => {
    document.querySelectorAll('.tab').forEach(x => x.classList.remove('active'));
    document.querySelectorAll('.panel').forEach(x => x.classList.remove('active'));
    t.classList.add('active');
    document.getElementById('panel-' + t.dataset.tab).classList.add('active');
    if (t.dataset.tab === 'gps') initMap();
    if (t.dataset.tab === 'lidar' && _records.length) drawLidar(+document.getElementById('frame-slider').value);
  });
});

// ── file list ──────────────────────────────────────────────────────────────────
async function loadFileList() {
  const res = await fetch('/api/files');
  const files = await res.json();
  const el = document.getElementById('file-list');
  if (!files.length) {
    el.innerHTML = '<div id="no-files">No .jsonl files found</div>';
    return;
  }
  el.innerHTML = files.map(f =>
    `<div class="file-entry" data-path="${f.path}">
      <div class="file-name">${f.name}</div>
      <div class="file-meta">${f.size_kb} KB</div>
    </div>`
  ).join('');
  el.querySelectorAll('.file-entry').forEach(e => {
    e.addEventListener('click', () => loadLog(e.dataset.path, e));
  });
}

// ── load log ──────────────────────────────────────────────────────────────────
async function loadLog(path, entryEl) {
  document.getElementById('loading').style.display = 'flex';
  try {
    const res  = await fetch('/api/load?path=' + encodeURIComponent(path));
    const data = await res.json();
    if (data.error) { alert(data.error); return; }

    _currentPath = path;
    _records     = data.records;

    document.querySelectorAll('.file-entry').forEach(e => e.classList.remove('active'));
    if (entryEl) entryEl.classList.add('active');

    showSummary(data.summary);
    buildCharts(_records);
    drawModeTimeline(_records);

    const n = _records.length;
    document.getElementById('frame-slider').max = n - 1;
    document.getElementById('frame-slider').value = 0;
    document.getElementById('insp-slider').max = n - 1;
    document.getElementById('insp-slider').value = 0;
    drawLidar(0);
    showFrame(0);

    if (_map) updateGpsLayer(_records);
  } finally {
    document.getElementById('loading').style.display = 'none';
  }
}

// ── summary ───────────────────────────────────────────────────────────────────
function showSummary(s) {
  const modes = Object.entries(s.mode_counts || {})
    .map(([k,v]) => `${k}: ${v}`).join(' | ');
  document.getElementById('summary').innerHTML = `
    <div class="badge">Records: <span>${s.count}</span></div>
    <div class="badge">Duration: <span>${s.duration_s}s</span></div>
    <div class="badge">Rate: <span>${s.hz} Hz</span></div>
    <div class="badge">GPS frames: <span>${s.gps_frames}</span></div>
    <div class="badge">Modes: <span>${modes}</span></div>
  `;
}

// ── charts ────────────────────────────────────────────────────────────────────
const MODE_COLORS = {MANUAL:'#2a9d8f', AUTO:'#e9c46a', ESTOP:'#e76f51'};

function _downsample(arr, maxPts) {
  if (arr.length <= maxPts) return arr;
  const step = arr.length / maxPts;
  return Array.from({length: maxPts}, (_, i) => arr[Math.floor(i * step)]);
}

function _mkLabels(recs) {
  const t0 = recs[0].ts;
  return recs.map(r => (r.ts - t0).toFixed(1) + 's');
}

function _mkChart(id, datasets, labels, opts = {}) {
  if (_charts[id]) { _charts[id].destroy(); delete _charts[id]; }
  const ctx = document.getElementById(id).getContext('2d');
  _charts[id] = new Chart(ctx, {
    type: 'line',
    data: { labels, datasets },
    options: {
      animation: false,
      responsive: true,
      maintainAspectRatio: false,
      plugins: { legend: { labels: { color: '#888', font: { size: 10 } } } },
      scales: {
        x: { ticks: { color: '#555', maxTicksLimit: 10, font: { size: 9 } }, grid: { color: '#222' } },
        y: { ticks: { color: '#555', font: { size: 9 } }, grid: { color: '#222' }, ...opts.y },
      },
      elements: { point: { radius: 0 }, line: { borderWidth: 1.5 } },
    }
  });
}

function buildCharts(recs) {
  if (!recs.length) return;
  const MAX = 800;
  const ds  = _downsample(recs, MAX);
  const lbl = _mkLabels(ds);

  // Drive
  _mkChart('chart-drive', [
    { label:'Left',  data: ds.map(r => r.drive?.left  ?? 0), borderColor:'#4a9eff', fill:false },
    { label:'Right', data: ds.map(r => r.drive?.right ?? 0), borderColor:'#ff6b6b', fill:false },
  ], lbl, { y: { min: -1, max: 1 } });

  // RC channels (CH1 steer idx 0, CH3 throttle idx 2)
  _mkChart('chart-rc', [
    { label:'Steer (CH1)',    data: ds.map(r => (r.rc_channels?.[0] ?? 1500)), borderColor:'#4a9eff', fill:false },
    { label:'Throttle (CH3)', data: ds.map(r => (r.rc_channels?.[2] ?? 1500)), borderColor:'#ff6b6b', fill:false },
  ], lbl, { y: { min: 900, max: 2100 } });

  // Voltage
  _mkChart('chart-voltage', [
    { label:'Voltage (V)', data: ds.map(r => r.telemetry?.voltage ?? null), borderColor:'#e9c46a', fill:false },
  ], lbl);

  // Heading
  _mkChart('chart-heading', [
    { label:'IMU Heading (°)', data: ds.map(r => r.telemetry?.heading ?? null), borderColor:'#8af', fill:false },
  ], lbl, { y: { min: 0, max: 360 } });

  // Temps
  _mkChart('chart-temp', [
    { label:'Board',       data: ds.map(r => r.telemetry?.board_temp ?? null), borderColor:'#f4a261', fill:false },
    { label:'Left motor',  data: ds.map(r => r.telemetry?.left_temp  ?? null), borderColor:'#e76f51', fill:false },
    { label:'Right motor', data: ds.map(r => r.telemetry?.right_temp ?? null), borderColor:'#ff6b6b', fill:false },
  ], lbl);

  // CPU
  _mkChart('chart-cpu', [
    { label:'CPU %',    data: ds.map(r => r.system?.cpu_pct  ?? null), borderColor:'#4a9eff', fill:false },
    { label:'CPU °C',   data: ds.map(r => r.system?.cpu_temp ?? null), borderColor:'#f4a261', fill:false },
  ], lbl);

  // GPS speed
  const gpsDs = _downsample(recs.filter(r => r.gps?.fix >= 1), MAX);
  if (gpsDs.length) {
    const gpsLbl = _mkLabels(gpsDs);
    _mkChart('chart-speed', [
      { label:'Speed (m/s)', data: gpsDs.map(r => r.gps?.speed ?? null), borderColor:'#2a9d8f', fill:false },
    ], gpsLbl);
  }
}

// ── mode timeline ─────────────────────────────────────────────────────────────
function drawModeTimeline(recs) {
  const canvas = document.getElementById('mode-canvas');
  canvas.width  = canvas.offsetWidth || 800;
  canvas.height = 36;
  const ctx = canvas.getContext('2d');
  const w = canvas.width, h = canvas.height;
  ctx.clearRect(0, 0, w, h);
  if (!recs.length) return;

  const t0 = recs[0].ts, t1 = recs[recs.length-1].ts, dt = t1 - t0 || 1;
  for (let i = 0; i < recs.length - 1; i++) {
    const r  = recs[i];
    const x0 = ((r.ts - t0) / dt) * w;
    const x1 = ((recs[i+1].ts - t0) / dt) * w;
    ctx.fillStyle = MODE_COLORS[r.mode] || '#444';
    ctx.fillRect(x0, 0, x1 - x0 + 1, h);
  }
  // legend
  let lx = 6;
  for (const [name, col] of Object.entries(MODE_COLORS)) {
    ctx.fillStyle = col; ctx.fillRect(lx, 10, 14, 14);
    ctx.fillStyle = '#ddd'; ctx.font = '10px monospace';
    ctx.fillText(name, lx + 17, 22);
    lx += 80;
  }
}

// ── GPS map ───────────────────────────────────────────────────────────────────
function initMap() {
  if (_map) return;
  _map = L.map('map', {preferCanvas: true}).setView([52.2, 0.1], 15);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
    { attribution:'© OpenStreetMap', maxZoom:20 }).addTo(_map);
  if (_records.length) updateGpsLayer(_records);
}

function updateGpsLayer(recs) {
  if (!_map) return;
  if (_gpsLayer) { _map.removeLayer(_gpsLayer); _gpsLayer = null; }

  const pts = recs
    .filter(r => r.gps?.fix >= 1 && r.gps?.lat && r.gps?.lon)
    .map(r => [r.gps.lat, r.gps.lon]);

  if (!pts.length) return;

  _gpsLayer = L.layerGroup().addTo(_map);

  // Colour track by mode
  let seg = [], lastMode = null;
  const flushSeg = mode => {
    if (seg.length > 1) {
      L.polyline(seg, {color: MODE_COLORS[mode] || '#8af', weight: 3, opacity: 0.85})
        .addTo(_gpsLayer);
    }
  };
  for (const r of recs.filter(r => r.gps?.fix >= 1 && r.gps?.lat && r.gps?.lon)) {
    if (r.mode !== lastMode && seg.length) { flushSeg(lastMode); seg = []; }
    seg.push([r.gps.lat, r.gps.lon]);
    lastMode = r.mode;
  }
  flushSeg(lastMode);

  // Start / end markers
  L.circleMarker(pts[0],  {radius:7, color:'#2a9d8f', fillColor:'#2a9d8f', fillOpacity:1}).addTo(_gpsLayer)
   .bindTooltip('Start');
  L.circleMarker(pts[pts.length-1], {radius:7, color:'#e76f51', fillColor:'#e76f51', fillOpacity:1}).addTo(_gpsLayer)
   .bindTooltip('End');

  _map.fitBounds(L.latLngBounds(pts).pad(0.1));
}

// ── LiDAR ─────────────────────────────────────────────────────────────────────
const _lidarCache = {};

async function drawLidar(idx) {
  const canvas = document.getElementById('lidar-canvas');
  const ctx    = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;
  const cx = W / 2, cy = H / 2, R = W / 2 - 20;

  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = '#0d0d0d';
  ctx.fillRect(0, 0, W, H);

  // Grid rings
  ctx.strokeStyle = '#222'; ctx.lineWidth = 1;
  for (const fr of [0.25, 0.5, 0.75, 1.0]) {
    ctx.beginPath(); ctx.arc(cx, cy, R * fr, 0, Math.PI*2); ctx.stroke();
  }
  ctx.beginPath(); ctx.moveTo(cx - R - 5, cy); ctx.lineTo(cx + R + 5, cy); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(cx, cy - R - 5); ctx.lineTo(cx, cy + R + 5); ctx.stroke();

  document.getElementById('frame-label').textContent = `Frame ${idx}`;

  if (!_currentPath || !_records.length) return;

  // Fetch full frame (has LiDAR data)
  let frame = _lidarCache[idx];
  if (!frame) {
    try {
      const res = await fetch(`/api/frame?path=${encodeURIComponent(_currentPath)}&idx=${idx}`);
      frame = await res.json();
      _lidarCache[idx] = frame;
    } catch (e) { return; }
  }

  document.getElementById('frame-ts').textContent = frame.ts_iso || '';

  const lidar = frame.lidar;
  if (!lidar || !lidar.angles || !lidar.angles.length) {
    ctx.fillStyle = '#444'; ctx.font = '13px monospace'; ctx.textAlign = 'center';
    ctx.fillText('No LiDAR data', cx, cy); return;
  }

  const angles = lidar.angles, dists = lidar.distances;
  const maxDist = Math.max(...dists.filter(d => d > 0)) || 5000;

  ctx.fillStyle = '#00ff88';
  for (let i = 0; i < angles.length; i++) {
    const d = dists[i];
    if (d <= 0) continue;
    const rad = (angles[i] - 90) * Math.PI / 180;
    const r   = (d / maxDist) * R;
    ctx.beginPath();
    ctx.arc(cx + r * Math.cos(rad), cy + r * Math.sin(rad), 2, 0, Math.PI*2);
    ctx.fill();
  }

  // Range label
  ctx.fillStyle = '#555'; ctx.font = '10px monospace'; ctx.textAlign = 'left';
  ctx.fillText(`max ${(maxDist/1000).toFixed(1)} m`, 8, H - 8);

  // Robot dot
  ctx.fillStyle = '#fff';
  ctx.beginPath(); ctx.arc(cx, cy, 5, 0, Math.PI*2); ctx.fill();
}

document.getElementById('frame-slider').addEventListener('input', e => {
  drawLidar(+e.target.value);
});

// ── Frame inspector ───────────────────────────────────────────────────────────
async function showFrame(idx) {
  document.getElementById('insp-label').textContent = `Frame ${idx}`;
  if (!_currentPath) return;
  let frame = _lidarCache[idx];
  if (!frame) {
    try {
      const res = await fetch(`/api/frame?path=${encodeURIComponent(_currentPath)}&idx=${idx}`);
      frame = await res.json();
      _lidarCache[idx] = frame;
    } catch (e) { return; }
  }
  // Render without lidar bulk
  const display = JSON.parse(JSON.stringify(frame));
  if (display.lidar) { display.lidar = { ts: display.lidar.ts, points: display.lidar.angles?.length ?? 0 }; }
  document.getElementById('frame-json').textContent = JSON.stringify(display, null, 2);
}

document.getElementById('insp-slider').addEventListener('input', e => {
  showFrame(+e.target.value);
});

// Sync sliders
document.getElementById('frame-slider').addEventListener('input', e => {
  document.getElementById('insp-slider').value = e.target.value;
});
document.getElementById('insp-slider').addEventListener('input', e => {
  document.getElementById('frame-slider').value = e.target.value;
  drawLidar(+e.target.value);
});

// ── Init ───────────────────────────────────────────────────────────────────────
loadFileList();
</script>
</body>
</html>
"""

# ── Entry point ────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='HackyRacingRobot data log viewer')
    parser.add_argument('--dir',  default='', help='Directory containing .jsonl files')
    parser.add_argument('--port', type=int, default=5004, help='Web server port (default 5004)')
    parser.add_argument('--host', default='0.0.0.0')
    args = parser.parse_args()

    if args.dir:
        _log_dir = args.dir
    print(f"Data log viewer → http://localhost:{args.port}/")
    print(f"Log directory   : {_log_dir}")
    from flask import cli as _fc; _fc.show_server_banner = lambda *a, **k: None
    app.run(host=args.host, port=args.port, debug=False)
