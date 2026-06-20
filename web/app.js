// Tactical Radar Companion Script
// Parse dynamic host/port from URL query params (useful for custom ports or remote HUD hosts)
const urlParams = new URLSearchParams(window.location.search);
let wsHost = urlParams.get("host") || window.location.hostname || "127.0.0.1";
if (wsHost === "localhost" || wsHost === "[::1]") {
    wsHost = "127.0.0.1";
}
let wsPort = urlParams.get("port") || "8085";
let wsUrl = `ws://${wsHost}:${wsPort}`;
let ws = null;
let reconnectTimer = null;

async function initConnection() {
    try {
        const response = await fetch("/config");
        if (response.ok) {
            const config = await response.json();
            let resolvedHost = config.ws_host || "127.0.0.1";
            if (resolvedHost === "localhost" || resolvedHost === "[::1]" || resolvedHost === "0.0.0.0") {
                resolvedHost = window.location.hostname || "127.0.0.1";
                if (resolvedHost === "localhost" || resolvedHost === "[::1]") {
                    resolvedHost = "127.0.0.1";
                }
            }
            const resolvedPort = config.ws_port || "8085";
            const finalHost = urlParams.get("host") || resolvedHost;
            const finalPort = urlParams.get("port") || resolvedPort;
            wsUrl = `ws://${finalHost}:${finalPort}`;
        }
    } catch (e) {
        console.log("Could not fetch /config, using default connection settings", e);
    }
    connect();
}

// TUI states mirrored
let systemStatus = "OFFLINE";
let clippingRate = 0.0;
let cancellationRatio = 0.0;
let ellipseMode = "None";
let activeTowers = [];
let targets = [];
let transients = [];
let latestWaterfallRow = [];

// Selected target
let selectedTargetId = null;

// DOM Elements
const statusEl = document.getElementById("sys-status");
// Phase 3 & 4 Controls & Displays
const cicModeSelect = document.getElementById("cic-mode-select");
const stareModeToggle = document.getElementById("stare-mode-toggle");
const stareXEl = document.getElementById("stare-x");
const stareYEl = document.getElementById("stare-y");
const stareZEl = document.getElementById("stare-z");
const ghostMicToggle = document.getElementById("ghost-mic-toggle");
const ghostMicGain = document.getElementById("ghost-mic-gain");
const ghostMicGainVal = document.getElementById("ghost-mic-gain-val");
const cepstrumCanvas = document.getElementById("cepstrum-scope");
const cepstrumCtx = cepstrumCanvas ? cepstrumCanvas.getContext("2d") : null;
const respirationEl = document.getElementById("telemetry-respiration");
const payloadEl = document.getElementById("telemetry-payload");
const clippingEl = document.getElementById("sys-clipping");
const cancellationEl = document.getElementById("sys-cancellation");
const ellipsesEl = document.getElementById("sys-ellipses");
const telemetryRowsEl = document.getElementById("telemetry-rows");
const logsEl = document.getElementById("events-log");

// Canvases
const radarCanvas = document.getElementById("radar-scope");
const radarCtx = radarCanvas.getContext("2d");
const waterfallCanvas = document.getElementById("waterfall-scope");
const waterfallCtx = waterfallCanvas.getContext("2d");
const elevationCanvas = document.getElementById("elevation-profile");
const elevationCtx = elevationCanvas.getContext("2d");
const jemCanvas = document.getElementById("jem-spectrum");
const jemCtx = jemCanvas.getContext("2d");

const fftSpectrogramCanvas = document.getElementById("fft-spectrogram");
const fftSpectrogramCtx = fftSpectrogramCanvas ? fftSpectrogramCanvas.getContext("2d") : null;
const iqConstellationCanvas = document.getElementById("iq-constellation");
const iqConstellationCtx = iqConstellationCanvas ? iqConstellationCanvas.getContext("2d") : null;

// Antenna Alignment & JEM Spectrogram Globals
const alignerCanvas = document.getElementById("antenna-aligner");
const alignerCtx = alignerCanvas ? alignerCanvas.getContext("2d") : null;
const multipathCanvas = document.getElementById("multipath-scope");
const multipathCtx = multipathCanvas ? multipathCanvas.getContext("2d") : null;

let antennaHeading = 0;
let jemViewMode = "FFT"; // "FFT" or "SPECTROGRAM"
let microDopplerHistory = [];
let lastJemTargetId = null;
let jemOffscreenCanvas = null;
let jemOffscreenCtx = null;

const jemTemplates = {
    'none': { name: "No Template Selected", desc: "", lines: [], envelope: null },
    
    // DRONES
    'dji_mavic': { name: "DJI Mavic (Small Quad)", desc: "High RPM (6000-8000), 2-blade props", lines: [-200, -100, 100, 200], envelope: { span: 250, shape: "steep" } },
    'dji_inspire': { name: "DJI Inspire (Large Quad)", desc: "Medium RPM (4000-6000), 2-blade props", lines: [-150, -75, 75, 150], envelope: { span: 180, shape: "steep" } },
    'fpv_racer': { name: "Custom FPV Racer", desc: "Extreme RPM (15000+), 3-blade props", lines: [-300, -150, 150, 300], envelope: { span: 400, shape: "gradual" } },

    // HELICOPTERS
    'heli_light': { name: "Light Heli (Bell 206)", desc: "Main ~6.5 Hz, Tail ~40 Hz", lines: [-80, -40, -6.5, 6.5, 40, 80], envelope: { span: 100, shape: "sharp" } },
    'heli_heavy': { name: "Heavy Lift (CH-47)", desc: "Tandem, ~3.7 Hz BPF, Huge RCS", lines: [-11.1, -7.4, -3.7, 3.7, 7.4, 11.1], envelope: { span: 40, shape: "wide" } },

    // PROPELLERS
    'prop_cessna': { name: "Cessna 172 (Single Prop)", desc: "2-blade prop, 2400 RPM (80 Hz BPF)", lines: [-320, -240, -160, -80, 80, 160, 240, 320], envelope: { span: 350, shape: "gradual" } },
    'prop_c130': { name: "C-130 Hercules", desc: "4-blade turboprop (66 Hz BPF)", lines: [-264, -198, -132, -66, 66, 132, 198, 264], envelope: { span: 300, shape: "sharp" } },

    // TURBOFANS (JETS)
    'jet_737': { name: "Boeing 737 (Turbofan)", desc: "CFM56 (BPF 1920 Hz -> Aliased to 80 Hz)", lines: [-80, 80], envelope: { span: 200, shape: "turbofan" } },
    'jet_fighter': { name: "F-16 Fighter", desc: "F110 Low Bypass", lines: [-150, -45, 45, 150], envelope: { span: 400, shape: "turbofan" } }
};
let currentJemTemplate = 'none';

// 3D View State
let yaw3d = -0.6;
let pitch3d = 0.5;
let isDragging3d = false;
let previousMousePosition = { x: 0, y: 0 };

// Target History Cache
const targetHistoryCache = new Map();

// Sweep State
let sweepAngle = 0;
const sweepSpeed = 0.015; // rad per frame

// Log entries
let maxLogs = 50;

function addLog(text, type = "system-msg") {
    const entry = document.createElement("div");
    entry.className = `log-entry ${type}`;
    const now = new Date();
    const timeStr = now.toTimeString().split(" ")[0];
    entry.innerText = `[${timeStr}] ${text}`;
    logsEl.appendChild(entry);
    logsEl.scrollTop = logsEl.scrollHeight;

    while (logsEl.childElementCount > maxLogs) {
        logsEl.removeChild(logsEl.firstChild);
    }
}

// WebSocket Connection
function connect() {
    addLog(`Establishing connection to Rust server on ${wsUrl}...`, "system-msg");
    ws = new WebSocket(wsUrl);
    ws.binaryType = "arraybuffer";

    ws.onopen = () => {
        statusEl.innerText = "CONNECTED";
        statusEl.className = "status-active";
        addLog("Connected to Passive Radar DSP Pipeline.", "system-msg");
        if (reconnectTimer) {
            clearInterval(reconnectTimer);
            reconnectTimer = null;
        }
    };

    ws.onmessage = (event) => {
        if (event.data instanceof ArrayBuffer) {
            const int16View = new Int16Array(event.data);
            const floatData = new Float32Array(int16View.length);
            for (let i = 0; i < int16View.length; i++) {
                floatData[i] = int16View[i] / 32768.0;
            }
            playAudioChunk(floatData);
            return;
        }

        if (typeof Blob !== "undefined" && event.data instanceof Blob) {
            event.data.arrayBuffer().then(buf => {
                const int16View = new Int16Array(buf);
                const floatData = new Float32Array(int16View.length);
                for (let i = 0; i < int16View.length; i++) {
                    floatData[i] = int16View[i] / 32768.0;
                }
                playAudioChunk(floatData);
            });
            return;
        }

        try {
            const data = JSON.parse(event.data);
            if (data.targets === undefined && data.waterfall_row === undefined) {
                handleTerminalResponse(data);
            } else {
                handleTelemetry(data);
            }
        } catch (e) {
            console.error("Failed to parse WebSocket JSON payload", e);
        }
    };

    ws.onclose = () => {
        statusEl.innerText = "OFFLINE";
        statusEl.className = "status-offline";
        addLog("Disconnected from server. Retrying connection...", "system-msg");
        targets = [];
        updateTelemetryTable();
        
        if (!reconnectTimer) {
            reconnectTimer = setInterval(connect, 3000);
        }
    };

    ws.onerror = () => {
        ws.close();
    };
}

// Process Incoming Telemetry
function handleTelemetry(data) {
    if (data.center_freq !== undefined) {
        centerFreq = data.center_freq;
    }
    clippingRate = data.clipping_rate || 0.0;
    cancellationRatio = data.cancellation_db || 0.0;
    ellipseMode = data.ellipse_mode || "None";
    activeTowers = data.active_towers || [];
    
    // Process new transients to print log alerts
    const newTransients = data.transients || [];
    for (let i = newTransients.length - 1; i >= 0; i--) {
        const e = newTransients[i];
        const key = `${e.timestamp}_${e.frequency_hz}`;
        const alreadyLogged = transients.some(t => `${t.timestamp}_${t.frequency_hz}` === key);
        if (!alreadyLogged) {
            let logMsg = `METEOR TRAIL DETECTED: Type=${e.classification}, Doppler=${e.frequency_hz.toFixed(1)} Hz, SNR=${e.snr_db.toFixed(1)} dB`;
            if (e.tec !== undefined && e.tec !== null) {
                logMsg += `, TEC=${e.tec.toFixed(3)} TECU`;
            }
            addLog(logMsg, "meteor-msg");
            
            // Speak meteor alert
            if (!announcedMeteors.has(key)) {
                announcedMeteors.add(key);
                if (announcedMeteors.size > 100) {
                    const firstKey = announcedMeteors.values().next().value;
                    announcedMeteors.delete(firstKey);
                }
                speakVoiceAlert("Meteor trail detected");
            }
            
            // Trigger screen shake only if enabled
            if (data.screen_shake) {
                const hudOverlay = document.querySelector(".hud-overlay");
                if (hudOverlay) {
                    hudOverlay.classList.add("shake-active");
                    hudOverlay.addEventListener("animationend", () => {
                        hudOverlay.classList.remove("shake-active");
                    }, { once: true });
                }
            }
        }
    }
    transients = newTransients;

    // Detect target acquisition/loss for logging
    const newTargets = data.targets || [];
    newTargets.forEach(nt => {
        const existing = targets.find(t => t.id === nt.id);
        if (!existing) {
            addLog(`ACQUIRED LOCK: Target ${nt.callsign || 'T' + nt.id} [${nt.classification || 'Unknown'}] at alt ${(nt.pos_enu[2]/1000).toFixed(1)}km`, "target-msg");
            
            // Speak lock alert
            if (!announcedLocks.has(nt.id)) {
                announcedLocks.add(nt.id);
                speakVoiceAlert(`Lock acquired on target ${nt.id}`);
            }
        }
    });
    targets.forEach(t => {
        const stillExists = newTargets.find(nt => nt.id === t.id);
        if (!stillExists) {
            addLog(`LOST LOCK: Target ${t.callsign || 'T' + t.id} has exited radar envelope.`, "system-msg");
            announcedLocks.delete(t.id);
            if (selectedTargetId === t.id) {
                selectedTargetId = null;
            }
        }
    });

    targets = newTargets;
    latestWaterfallRow = data.waterfall_row || [];

    // Sync the antenna heading slider position and text value from telemetry
    const antennaHeadingSlider = document.getElementById("antenna-heading");
    const antennaHeadingVal = document.getElementById("antenna-heading-val");
    if (data.antenna_heading !== undefined) {
        if (antennaHeadingSlider) {
            antennaHeadingSlider.value = data.antenna_heading;
        }
        if (antennaHeadingVal) {
            antennaHeadingVal.innerText = `${data.antenna_heading.toFixed(0)}°`;
        }
        antennaHeading = data.antenna_heading;
    }

    // Update target-specific microDopplerHistory and offscreen canvas
    const selTarget = targets.find(target => target.id === selectedTargetId);
    if (selTarget && selTarget.jem_fft_mag && selTarget.jem_fft_mag.length > 0) {
        const w = jemCanvas ? jemCanvas.width : 0;
        const h = jemCanvas ? jemCanvas.height : 0;
        if (selectedTargetId !== lastJemTargetId) {
            microDopplerHistory = [];
            lastJemTargetId = selectedTargetId;
            if (w > 0 && h > 0) {
                // Clear offscreen canvas
                if (!jemOffscreenCanvas || jemOffscreenCanvas.width !== w || jemOffscreenCanvas.height !== h) {
                    jemOffscreenCanvas = document.createElement("canvas");
                    jemOffscreenCanvas.width = w;
                    jemOffscreenCanvas.height = h;
                    jemOffscreenCtx = jemOffscreenCanvas.getContext("2d");
                }
                jemOffscreenCtx.fillStyle = "rgba(3, 9, 20, 1.0)";
                jemOffscreenCtx.fillRect(0, 0, w, h);
            }
        }
        microDopplerHistory.push(selTarget.jem_fft_mag);
        if (microDopplerHistory.length > 150) {
            microDopplerHistory.shift();
        }
        
        // Render to offscreen canvas
        if (w > 0 && h > 0) {
            shiftAndDrawJemWaterfall(selTarget.jem_fft_mag);
        }
    } else {
        microDopplerHistory = [];
        lastJemTargetId = null;
    }

    // Update target history cache
    targets.forEach(t => {
        if (!targetHistoryCache.has(t.id)) {
            targetHistoryCache.set(t.id, []);
        }
        const hist = targetHistoryCache.get(t.id);
        hist.push([t.pos_enu[0] / 1000, t.pos_enu[1] / 1000, t.pos_enu[2] / 1000]);
        if (hist.length > 100) {
            hist.shift();
        }
    });
    // Clean up stale targets
    for (const key of targetHistoryCache.keys()) {
        if (!targets.some(t => t.id === key)) {
            targetHistoryCache.delete(key);
        }
    }

    // Update Text elements
    clippingEl.innerText = `${(clippingRate * 100).toFixed(1)}%`;
    cancellationEl.innerText = `${Math.max(0.0, cancellationRatio).toFixed(1)} dB`;
    ellipsesEl.innerText = ellipseMode.toUpperCase();

    // Update Airspace summary
    let planes = 0;
    let drones = 0;
    let other = 0;
    targets.forEach(t => {
        const classStr = (t.classification || "").toLowerCase();
        const callStr = (t.callsign || "").toLowerCase();
        if (classStr.includes("drone") || classStr.includes("uav") || callStr.includes("drn")) {
            drones++;
        } else if (classStr.includes("plane") || classStr.includes("b78") || classStr.includes("commercial") || callStr.includes("aal")) {
            planes++;
        } else {
            other++;
        }
    });

    const sumDensity = document.getElementById("sum-density");
    if (sumDensity) {
        if (targets.length === 0) {
            sumDensity.innerText = "CLEAR";
            sumDensity.style.color = "var(--accent-green)";
        } else if (targets.length <= 2) {
            sumDensity.innerText = "LOW DENSITY";
            sumDensity.style.color = "var(--accent-green)";
        } else if (targets.length <= 4) {
            sumDensity.innerText = "MODERATE";
            sumDensity.style.color = "var(--accent-yellow)";
        } else {
            sumDensity.innerText = "HIGH DENSITY";
            sumDensity.style.color = "#ff1744";
        }
    }

    const sumTotalTracks = document.getElementById("sum-total-tracks");
    if (sumTotalTracks) {
        sumTotalTracks.innerText = targets.length;
    }

    const sumBreakdown = document.getElementById("sum-breakdown");
    if (sumBreakdown) {
        sumBreakdown.innerHTML = `PLANES: <span style="color:var(--accent-cyan); font-weight:bold">${planes}</span> | DRONES: <span style="color:var(--accent-yellow); font-weight:bold">${drones}</span> | OTHER: <span style="color:var(--accent-orange); font-weight:bold">${other}</span>`;
    }

    // Update live status indicators
    const sdrStatusEl = document.getElementById("sdr-status");
    if (sdrStatusEl && data.sdr_alive !== undefined) {
        sdrStatusEl.innerText = data.sdr_alive ? "ACTIVE" : "OFFLINE";
        sdrStatusEl.className = data.sdr_alive ? "status-active" : "status-offline";
    }

    const sdrFreqEl = document.getElementById("sdr-frequency");
    if (sdrFreqEl && data.center_freq !== undefined) {
        sdrFreqEl.innerText = `${(data.center_freq / 1e6).toFixed(3)} MHz`;
    }

    const sdrRateEl = document.getElementById("sdr-sample-rate");
    if (sdrRateEl && data.sample_rate !== undefined) {
        sdrRateEl.innerText = `${(data.sample_rate / 1e6).toFixed(3)} MSPS`;
    }

    const sdrOverflowEl = document.getElementById("sdr-overflow");
    if (sdrOverflowEl && data.overflow_alarm !== undefined) {
        if (data.overflow_alarm) {
            sdrOverflowEl.innerText = "ALARM";
            sdrOverflowEl.className = "alarm-active";
        } else {
            sdrOverflowEl.innerText = "CLEAR";
            sdrOverflowEl.className = "status-active";
        }
    }

    // Sync Slider
    if (data.dsp_threshold !== undefined) {
        const slider = document.getElementById("dsp-threshold");
        const label = document.getElementById("dsp-threshold-val");
        if (slider && label && document.activeElement !== slider) {
            slider.value = data.dsp_threshold;
            label.innerText = `${data.dsp_threshold.toFixed(1)} dB`;
        }
    }

    // Sync control states from telemetry (if not focused)
    const sdrGainInput = document.getElementById("sdr-gain");
    const sdrGainVal = document.getElementById("sdr-gain-val");
    if (data.sdr_gain !== undefined && sdrGainInput && document.activeElement !== sdrGainInput) {
        sdrGainInput.value = data.sdr_gain;
        if (sdrGainVal) sdrGainVal.innerText = `${data.sdr_gain.toFixed(0)} dB`;
    }

    const sdrAgcInput = document.getElementById("sdr-agc");
    if (data.software_agc !== undefined && sdrAgcInput) {
        sdrAgcInput.checked = data.software_agc;
    }

    const sdrOffsetInput = document.getElementById("sdr-offset");
    if (data.sdr_offset !== undefined && sdrOffsetInput && document.activeElement !== sdrOffsetInput) {
        sdrOffsetInput.value = data.sdr_offset;
    }

    const sdrDcBlockInput = document.getElementById("sdr-dc-block");
    if (data.sdr_dc_block !== undefined && sdrDcBlockInput && document.activeElement !== sdrDcBlockInput) {
        sdrDcBlockInput.checked = data.sdr_dc_block;
    }
    const sdrOneBitInput = document.getElementById("sdr-one-bit");
    if (data.one_bit_mode !== undefined && sdrOneBitInput && document.activeElement !== sdrOneBitInput) {
        sdrOneBitInput.checked = data.one_bit_mode;
    }

    const unconfirmedToggle = document.getElementById("unconfirmed-toggle");
    if (data.show_unconfirmed !== undefined && unconfirmedToggle && document.activeElement !== unconfirmedToggle) {
        unconfirmedToggle.checked = data.show_unconfirmed;
    }

    const shakeToggle = document.getElementById("shake-toggle");
    if (data.screen_shake !== undefined && shakeToggle && document.activeElement !== shakeToggle) {
        shakeToggle.checked = data.screen_shake;
    }

    // Render components
    if (selTarget) {
        if (cicModeSelect && document.activeElement !== cicModeSelect) {
            cicModeSelect.value = selTarget.cic_mode || "Seismic";
        }
        if (stareModeToggle && document.activeElement !== stareModeToggle) {
            stareModeToggle.checked = !!selTarget.stare_mode_active;
        }
        if (stareXEl && document.activeElement !== stareXEl) {
            stareXEl.value = (selTarget.pos_enu && selTarget.pos_enu[0] !== undefined) ? selTarget.pos_enu[0].toFixed(1) : 0;
        }
        if (stareYEl && document.activeElement !== stareYEl) {
            stareYEl.value = (selTarget.pos_enu && selTarget.pos_enu[1] !== undefined) ? selTarget.pos_enu[1].toFixed(1) : 0;
        }
        if (stareZEl && document.activeElement !== stareZEl) {
            stareZEl.value = (selTarget.pos_enu && selTarget.pos_enu[2] !== undefined) ? selTarget.pos_enu[2].toFixed(1) : 0;
        }
        if (respirationEl) {
            respirationEl.innerText = (selTarget.respiration_rate !== undefined && selTarget.respiration_rate !== null)
                ? `${selTarget.respiration_rate.toFixed(2)} Hz`
                : "N/A";
        }
        if (payloadEl) {
            payloadEl.innerText = selTarget.payload_class || "N/A";
        }
    } else {
        if (respirationEl) respirationEl.innerText = "N/A";
        if (payloadEl) payloadEl.innerText = "N/A";
    }

    updateTelemetryTable();
    drawWaterfallRow();

    if (data.surveillance_fft) {
        drawFftSpectrogram(data.surveillance_fft);
    }
    if (data.constellation_points) {
        drawIqConstellation(data.constellation_points);
    }
    if (data.multipath_profile) {
        drawMultipathProfile(data.multipath_profile, data.multipath_peak_refined);
    }

    // Update Tactical Records/High Scores
    if (data.tactical_records) {
        const tr = data.tactical_records;
        
        const fastestPlaneEl = document.getElementById("record-fastest-plane");
        if (fastestPlaneEl) {
            if (tr.fastest_plane) {
                const val = tr.fastest_plane.value;
                fastestPlaneEl.innerText = `${val.toFixed(1)} m/s (${tr.fastest_plane.callsign || 'T' + tr.fastest_plane.target_id} - ${tr.fastest_plane.classification})`;
            } else {
                fastestPlaneEl.innerText = "N/A";
            }
        }
        
        const highestDroneEl = document.getElementById("record-highest-drone");
        if (highestDroneEl) {
            if (tr.highest_drone) {
                const val = tr.highest_drone.value;
                highestDroneEl.innerText = `${val.toFixed(1)} m (${tr.highest_drone.callsign || 'T' + tr.highest_drone.target_id} - ${tr.highest_drone.classification})`;
            } else {
                highestDroneEl.innerText = "N/A";
            }
        }
        
        const closestTargetEl = document.getElementById("record-closest-target");
        if (closestTargetEl) {
            if (tr.closest_target) {
                const val = tr.closest_target.value / 1000.0;
                closestTargetEl.innerText = `${val.toFixed(1)} km (${tr.closest_target.callsign || 'T' + tr.closest_target.target_id} - ${tr.closest_target.classification})`;
            } else {
                closestTargetEl.innerText = "N/A";
            }
        }
        
        const maxTracksEl = document.getElementById("record-max-tracks");
        if (maxTracksEl) {
            maxTracksEl.innerText = tr.max_simultaneous_tracks || "0";
        }
        
        const maxCancellationEl = document.getElementById("record-max-cancellation");
        if (maxCancellationEl) {
            const val = tr.max_cancellation || 0.0;
            maxCancellationEl.innerText = `${val.toFixed(1)} dB`;
        }
    }
}

// Update Telemetry Grid
function updateTelemetryTable() {
    if (targets.length === 0) {
        telemetryRowsEl.innerHTML = `<tr><td colspan="6" class="no-data">NO TARGETS ACQUIRED</td></tr>`;
        return;
    }

    let rowsHtml = "";
    targets.forEach(t => {
        const isSelected = selectedTargetId === t.id;
        const altKm = (t.pos_enu[2] / 1000).toFixed(2);
        const speedMps = t.speed_mps.toFixed(1);
        const stateClass = `state-${t.state.toLowerCase()}`;
        const rowClass = isSelected ? "selected-row" : "";
        const towerStr = t.tracking_towers.join(", ") || "None";
        
        rowsHtml += `
            <tr class="${rowClass}" onclick="selectTarget(${t.id})" style="cursor: pointer; ${isSelected ? 'background: rgba(0, 229, 255, 0.12); border-left: 3px solid #00e5ff;' : ''}">
                <td>${t.callsign || 'T' + t.id}</td>
                <td class="${stateClass}">${t.state.toUpperCase()}</td>
                <td>${t.classification || 'Acquiring...'}</td>
                <td>${altKm} km</td>
                <td>${speedMps} m/s</td>
                <td>${towerStr}</td>
            </tr>
        `;
    });
    telemetryRowsEl.innerHTML = rowsHtml;
}

window.selectTarget = function(id) {
    if (selectedTargetId === id) {
        selectedTargetId = null;
    } else {
        selectedTargetId = id;
    }
    updateTelemetryTable();
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ command: "SelectTarget", id: selectedTargetId }));
    }
};

// Canvas Resize handlers
function resizeCanvases() {
    const rParent = radarCanvas.parentElement;
    radarCanvas.width = rParent.clientWidth;
    radarCanvas.height = rParent.clientHeight;

    const wParent = waterfallCanvas.parentElement;
    const oldWidth = waterfallCanvas.width;
    const oldHeight = waterfallCanvas.height;
    
    let backupCanvas = null;
    if (oldWidth > 0 && oldHeight > 0) {
        backupCanvas = document.createElement("canvas");
        backupCanvas.width = oldWidth;
        backupCanvas.height = oldHeight;
        const backupCtx = backupCanvas.getContext("2d");
        backupCtx.drawImage(waterfallCanvas, 0, 0);
    }

    waterfallCanvas.width = wParent.clientWidth;
    waterfallCanvas.height = wParent.clientHeight;

    if (backupCanvas) {
        waterfallCtx.drawImage(backupCanvas, 0, 0, oldWidth, oldHeight, 0, 0, waterfallCanvas.width, waterfallCanvas.height);
    }
    waterfallCtx.imageSmoothingEnabled = false;

    if (elevationCanvas) {
        const eParent = elevationCanvas.parentElement;
        elevationCanvas.width = eParent.clientWidth;
        elevationCanvas.height = eParent.clientHeight;
    }

    if (jemCanvas) {
        const jParent = jemCanvas.parentElement;
        jemCanvas.width = jParent.clientWidth;
        jemCanvas.height = jParent.clientHeight;
    }
    if (fftSpectrogramCanvas) {
        const fParent = fftSpectrogramCanvas.parentElement;
        fftSpectrogramCanvas.width = fParent.clientWidth;
        fftSpectrogramCanvas.height = fParent.clientHeight;
    }
    if (iqConstellationCanvas) {
        const iParent = iqConstellationCanvas.parentElement;
        iqConstellationCanvas.width = iParent.clientWidth;
        iqConstellationCanvas.height = iParent.clientHeight;
    }
    if (alignerCanvas) {
        const aParent = alignerCanvas.parentElement;
        alignerCanvas.width = aParent.clientWidth;
        alignerCanvas.height = aParent.clientHeight;
    }
    if (multipathCanvas) {
        const mParent = multipathCanvas.parentElement;
        multipathCanvas.width = mParent.clientWidth;
        multipathCanvas.height = mParent.clientHeight;
    }
    if (cepstrumCanvas) {
        const cParent = cepstrumCanvas.parentElement;
        cepstrumCanvas.width = cParent.clientWidth;
        cepstrumCanvas.height = cParent.clientHeight;
    }
}
window.addEventListener("resize", resizeCanvases);

// Color Mappers
function getPlasmaColor(val) {
    // Normalise val (assumed dB ranging from -70 to -10) to 0.0 - 1.0
    const norm = Math.max(0, Math.min(1, (val + 70) / 60));
    
    // Simple Plasma scale approximations
    const r = Math.floor(Math.sin(norm * Math.PI / 2) * 255);
    const g = Math.floor(Math.sin(norm * Math.PI) * Math.sin(norm * Math.PI) * 150 + norm * 50);
    const b = Math.floor((1 - Math.sin(norm * Math.PI / 2)) * 150 + norm * 105);
    
    return `rgb(${r}, ${g}, ${b})`;
}

// Waterfall Scroll-Down Drawing
function drawWaterfallRow() {
    if (latestWaterfallRow.length === 0) return;

    const w = waterfallCanvas.width;
    const h = waterfallCanvas.height;

    // Shift waterfall down by 2 pixels using hardware accelerated drawImage
    waterfallCtx.drawImage(waterfallCanvas, 0, 0, w, h - 2, 0, 2, w, h - 2);

    // Render the new row at the top 2 pixels
    const numBins = latestWaterfallRow.length;
    const binWidth = w / numBins;

    for (let i = 0; i < numBins; i++) {
        const db = latestWaterfallRow[i];
        waterfallCtx.fillStyle = getPlasmaColor(db);
        waterfallCtx.fillRect(i * binWidth, 0, binWidth + 1, 2);
    }
}

// Neon Phosphor waterfall shader color mapper
function getPhosphorColor(val) {
    const db = 20 * Math.log10(Math.max(1e-5, val));
    const norm = Math.max(0, Math.min(1, (db + 60) / 60)); // map -60dB to 0dB
    const g = Math.floor(norm * 229);
    const b = Math.floor(norm * 255);
    return `rgba(0, ${g}, ${b}, ${norm})`;
}

function drawFftSpectrogram(fftData) {
    if (!fftSpectrogramCanvas || !fftSpectrogramCtx) return;
    const w = fftSpectrogramCanvas.width;
    const h = fftSpectrogramCanvas.height;
    if (w === 0 || h === 0) return;

    // Shift canvas down by 2 pixels using hardware accelerated drawImage
    fftSpectrogramCtx.drawImage(fftSpectrogramCanvas, 0, 0, w, h - 2, 0, 2, w, h - 2);

    const numBins = fftData.length;
    const binWidth = w / numBins;
    for (let i = 0; i < numBins; i++) {
        const val = fftData[i];
        fftSpectrogramCtx.fillStyle = getPhosphorColor(val);
        fftSpectrogramCtx.fillRect(i * binWidth, 0, binWidth + 1, 2);
    }
}

function drawIqConstellation(points) {
    if (!iqConstellationCanvas || !iqConstellationCtx) return;
    const w = iqConstellationCanvas.width;
    const h = iqConstellationCanvas.height;
    if (w === 0 || h === 0) return;

    iqConstellationCtx.fillStyle = "rgba(3, 9, 20, 0.45)";
    iqConstellationCtx.fillRect(0, 0, w, h);

    const cx = w / 2;
    const cy = h / 2;
    const scale = Math.min(cx, cy) * 0.8;

    // Draw unit polar scope grid lines
    iqConstellationCtx.strokeStyle = "rgba(0, 229, 255, 0.12)";
    iqConstellationCtx.lineWidth = 1;
    iqConstellationCtx.beginPath();
    iqConstellationCtx.moveTo(cx - scale, cy); iqConstellationCtx.lineTo(cx + scale, cy);
    iqConstellationCtx.moveTo(cx, cy - scale); iqConstellationCtx.lineTo(cx, cy + scale);
    iqConstellationCtx.arc(cx, cy, scale, 0, Math.PI * 2);
    iqConstellationCtx.stroke();

    // Render constellation dot points
    iqConstellationCtx.fillStyle = "rgba(0, 229, 255, 0.8)";
    points.forEach(pt => {
        const px = cx + pt[0] * scale;
        const py = cy - pt[1] * scale;
        iqConstellationCtx.beginPath();
        iqConstellationCtx.arc(px, py, 1.8, 0, Math.PI * 2);
        iqConstellationCtx.fill();
    });
}

function drawMultipathProfile(profile, refinedPeak) {
    if (!multipathCanvas || !multipathCtx) return;
    const w = multipathCanvas.width;
    const h = multipathCanvas.height;
    if (w === 0 || h === 0) return;

    // Clear
    multipathCtx.fillStyle = "rgba(3, 9, 20, 0.45)";
    multipathCtx.fillRect(0, 0, w, h);

    const paddingLeft = 40;
    const paddingRight = 15;
    const paddingTop = 20;
    const paddingBottom = 25;

    const graphWidth = w - paddingLeft - paddingRight;
    const graphHeight = h - paddingTop - paddingBottom;

    if (graphWidth <= 0 || graphHeight <= 0) return;

    // Draw grid
    multipathCtx.strokeStyle = "rgba(0, 229, 255, 0.08)";
    multipathCtx.lineWidth = 1;
    
    // Y grid (4 levels)
    for (let i = 1; i <= 4; i++) {
        const y = paddingTop + graphHeight * (1 - i / 4);
        multipathCtx.beginPath();
        multipathCtx.moveTo(paddingLeft, y);
        multipathCtx.lineTo(w - paddingRight, y);
        multipathCtx.stroke();
    }

    // X grid (every 10 bins)
    const numBins = profile.length;
    for (let i = 0; i < numBins; i += 10) {
        const x = paddingLeft + (i / (numBins - 1)) * graphWidth;
        multipathCtx.beginPath();
        multipathCtx.moveTo(x, paddingTop);
        multipathCtx.lineTo(x, h - paddingBottom);
        multipathCtx.stroke();
        
        const distKm = i * 1.171;
        multipathCtx.fillStyle = "rgba(0, 229, 255, 0.6)";
        multipathCtx.font = "8px Courier New";
        multipathCtx.textAlign = "center";
        multipathCtx.fillText(`${distKm.toFixed(0)}k`, x, h - 12);
    }

    // Find max value and index
    let maxIdx = 0;
    let maxVal = 1e-6;
    for (let i = 0; i < numBins; i++) {
        if (profile[i] > maxVal) {
            maxVal = profile[i];
            maxIdx = i;
        }
    }

    // Plot line
    multipathCtx.beginPath();
    for (let i = 0; i < numBins; i++) {
        const x = paddingLeft + (i / (numBins - 1)) * graphWidth;
        const normVal = profile[i] / maxVal;
        const y = paddingTop + graphHeight * (1 - normVal);
        if (i === 0) {
            multipathCtx.moveTo(x, y);
        } else {
            multipathCtx.lineTo(x, y);
        }
    }
    multipathCtx.strokeStyle = "rgba(0, 229, 255, 0.85)";
    multipathCtx.lineWidth = 1.5;
    
    // Glowing line
    multipathCtx.shadowColor = "rgba(0, 229, 255, 0.5)";
    multipathCtx.shadowBlur = 4;
    multipathCtx.stroke();
    multipathCtx.shadowBlur = 0; // reset

    // Fill area under gradient
    const gradient = multipathCtx.createLinearGradient(0, paddingTop, 0, h - paddingBottom);
    gradient.addColorStop(0, "rgba(0, 229, 255, 0.2)");
    gradient.addColorStop(1, "rgba(0, 229, 255, 0.0)");
    
    multipathCtx.beginPath();
    multipathCtx.moveTo(paddingLeft, h - paddingBottom);
    for (let i = 0; i < numBins; i++) {
        const x = paddingLeft + (i / (numBins - 1)) * graphWidth;
        const normVal = profile[i] / maxVal;
        const y = paddingTop + graphHeight * (1 - normVal);
        multipathCtx.lineTo(x, y);
    }
    multipathCtx.lineTo(w - paddingRight, h - paddingBottom);
    multipathCtx.closePath();
    multipathCtx.fillStyle = gradient;
    multipathCtx.fill();

    // Peak marker callout
    if (maxVal > 1e-5) {
        const peakIdx = (refinedPeak !== undefined && refinedPeak !== null) ? refinedPeak : maxIdx;
        const maxPx = paddingLeft + (peakIdx / (numBins - 1)) * graphWidth;
        const maxPy = paddingTop + graphHeight * (1 - 1.0);
        
        multipathCtx.fillStyle = "rgba(255, 59, 48, 0.85)"; // coral/red glow
        multipathCtx.beginPath();
        multipathCtx.arc(maxPx, maxPy, 3.5, 0, Math.PI * 2);
        multipathCtx.fill();

        multipathCtx.fillStyle = "#ffffff";
        multipathCtx.font = "bold 8px Courier New";
        multipathCtx.textAlign = peakIdx > numBins / 2 ? "right" : "left";
        const offsetSign = peakIdx > numBins / 2 ? -6 : 6;
        const maxDistKm = peakIdx * 1.171;
        multipathCtx.fillText(`${maxDistKm.toFixed(3)} km`, maxPx + offsetSign, maxPy + 10);
    }

    // Axis
    multipathCtx.strokeStyle = "rgba(0, 229, 255, 0.3)";
    multipathCtx.lineWidth = 1;
    multipathCtx.beginPath();
    multipathCtx.moveTo(paddingLeft, paddingTop);
    multipathCtx.lineTo(paddingLeft, h - paddingBottom);
    multipathCtx.lineTo(w - paddingRight, h - paddingBottom);
    multipathCtx.stroke();

    // Y Axis labels
    multipathCtx.fillStyle = "rgba(0, 229, 255, 0.6)";
    multipathCtx.font = "8px Courier New";
    multipathCtx.textAlign = "right";
    multipathCtx.fillText("MAX", paddingLeft - 5, paddingTop + 4);
    multipathCtx.fillText("0.5", paddingLeft - 5, paddingTop + graphHeight / 2 + 3);
    multipathCtx.fillText("0.0", paddingLeft - 5, h - paddingBottom + 2);
}


// Radar PPI Paint Loop
function drawRadarScope() {
    requestAnimationFrame(drawRadarScope);

    const w = radarCanvas.width;
    const h = radarCanvas.height;
    if (w === 0 || h === 0) return;

    const cx = w / 2;
    const cy = h / 2;
    const maxRadius = Math.min(cx, cy) * 0.95;

    // Standard PPI trace persistence
    radarCtx.fillStyle = "rgba(3, 9, 20, 0.18)";
    radarCtx.fillRect(0, 0, w, h);

    // Coordinate conversion helper (ENU km -> pixels)
    // Map bounds: [-70, 70] km
    const enuToPixel = (x_km, y_km) => {
        const px = cx + (x_km / 70) * maxRadius;
        const py = cy - (y_km / 70) * maxRadius; // invert Y for screen space
        return { x: px, y: py };
    };

    // 1. Draw Range Rings & Axes
    radarCtx.strokeStyle = "rgba(0, 229, 255, 0.1)";
    radarCtx.lineWidth = 1;
    
    // Axes
    radarCtx.beginPath();
    radarCtx.moveTo(cx - maxRadius, cy); radarCtx.lineTo(cx + maxRadius, cy);
    radarCtx.moveTo(cx, cy - maxRadius); radarCtx.lineTo(cx, cy + maxRadius);
    radarCtx.stroke();

    // Rings
    const ringRanges = [25, 50, 70];
    ringRanges.forEach(r => {
        const radius = (r / 70) * maxRadius;
        radarCtx.beginPath();
        radarCtx.arc(cx, cy, radius, 0, Math.PI * 2);
        radarCtx.stroke();

        // Print ring tags
        radarCtx.fillStyle = "rgba(0, 229, 255, 0.3)";
        radarCtx.font = "10px Share Tech Mono";
        radarCtx.fillText(`${r} km`, cx + 5, cy - radius + 12);
    });

    // Outer azimuth ticks
    radarCtx.strokeStyle = "rgba(0, 229, 255, 0.2)";
    for (let angle = 0; angle < 360; angle += 10) {
        const rad = (angle - 90) * Math.PI / 180;
        const startR = maxRadius;
        const endR = maxRadius - (angle % 30 === 0 ? 8 : 4);
        
        radarCtx.beginPath();
        radarCtx.moveTo(cx + Math.cos(rad) * startR, cy + Math.sin(rad) * startR);
        radarCtx.lineTo(cx + Math.cos(rad) * endR, cy + Math.sin(rad) * endR);
        radarCtx.stroke();

        if (angle % 30 === 0) {
            radarCtx.fillStyle = "rgba(0, 229, 255, 0.5)";
            radarCtx.font = "9px Orbitron";
            radarCtx.textAlign = "center";
            radarCtx.textBaseline = "middle";
            radarCtx.fillText(
                `${angle}°`, 
                cx + Math.cos(rad) * (startR + 12), 
                cy + Math.sin(rad) * (startR + 12)
            );
        }
    }

    // 1.1 Draw Bistatic Range Ellipses for Active Towers
    if (ellipseMode !== "None" && ellipseMode !== "NONE") {
        activeTowers.forEach((tow, idx) => {
            let showEllipse = false;
            if (ellipseMode === "All" || ellipseMode === "ALL") {
                showEllipse = true;
            } else if ((ellipseMode === "Selected" || ellipseMode === "SELECTED") && selectedTargetId !== null) {
                const selTarget = targets.find(t => t.id === selectedTargetId);
                if (selTarget && selTarget.tracking_towers) {
                    showEllipse = selTarget.tracking_towers.includes(tow.name);
                }
            }

            if (showEllipse) {
                const tx_x = tow.pos_enu[0] / 1000; // km
                const tx_y = tow.pos_enu[1] / 1000; // km
                const baseline = Math.sqrt(tx_x * tx_x + tx_y * tx_y);

                if (baseline >= 1.0) {
                    const xc = tx_x / 2;
                    const yc = tx_y / 2;
                    const cos_theta = tx_x / baseline;
                    const sin_theta = tx_y / baseline;

                    const ellipseColors = ["#e040fb", "#00e5ff", "#ffea00", "#ff5252", "#00e676", "#29b6f6"];
                    const color = ellipseColors[idx % ellipseColors.length];

                    radarCtx.save();
                    radarCtx.strokeStyle = color;
                    radarCtx.lineWidth = 1;
                    radarCtx.setLineDash([2, 4]);
                    
                    const ranges = [baseline + 20.0, baseline + 50.0];
                    ranges.forEach(r_b => {
                        const a_km = r_b / 2;
                        const b_km = Math.sqrt(Math.max(0, a_km * a_km - (baseline / 2) * (baseline / 2)));
                        
                        const aPixels = (a_km / 70) * maxRadius;
                        const bPixels = (b_km / 70) * maxRadius;
                        const centerPos = enuToPixel(xc, yc);
                        const phi = Math.atan2(tx_y, tx_x);

                        radarCtx.beginPath();
                        radarCtx.ellipse(centerPos.x, centerPos.y, aPixels, bPixels, -phi, 0, Math.PI * 2);
                        radarCtx.stroke();

                        // Print range label at the top vertex
                        const label_x = xc - b_km * sin_theta;
                        const label_y = yc + b_km * cos_theta;
                        const labelPos = enuToPixel(label_x, label_y);
                        
                        radarCtx.fillStyle = color;
                        radarCtx.font = "8px Share Tech Mono";
                        radarCtx.textAlign = "center";
                        radarCtx.fillText(`${r_b.toFixed(0)} km`, labelPos.x, labelPos.y - 2);
                    });
                    radarCtx.restore();
                }
            }
        });
    }

    // 2. Draw Receiver center (Rx)
    radarCtx.fillStyle = "#00e676";
    radarCtx.beginPath();
    radarCtx.arc(cx, cy, 3, 0, Math.PI * 2);
    radarCtx.fill();
    radarCtx.strokeStyle = "rgba(0, 230, 118, 0.5)";
    radarCtx.beginPath();
    radarCtx.arc(cx, cy, 8, 0, Math.PI * 2);
    radarCtx.stroke();

    // 3. Draw Transmitter Towers
    activeTowers.forEach(tow => {
        const pos = enuToPixel(tow.pos_enu[0] / 1000, tow.pos_enu[1] / 1000);
        radarCtx.fillStyle = "#e040fb";
        
        // Triangle vector symbol
        radarCtx.beginPath();
        radarCtx.moveTo(pos.x, pos.y - 6);
        radarCtx.lineTo(pos.x - 5, pos.y + 4);
        radarCtx.lineTo(pos.x + 5, pos.y + 4);
        radarCtx.closePath();
        radarCtx.fill();

        radarCtx.font = "9px Share Tech Mono";
        radarCtx.fillText(tow.name, pos.x + 8, pos.y + 3);
    });

    // 4. Draw Targets & Trails
    targets.forEach(t => {
        const isSelected = selectedTargetId === t.id;
        const pos = enuToPixel(t.pos_enu[0] / 1000, t.pos_enu[1] / 1000);
        
        // Target status color mapping
        let color = "#00e676";
        if (t.state === "Coasting") color = "#ff9100";
        if (t.state === "Suspect") color = "#ffea00";
        if (t.state === "Terminated") color = "#90a4ae";

        // 4.1 Draw 95% confidence EKF uncertainty ellipse
        if (t.ekf_cov && t.ekf_cov.length >= 3) {
            const p_xx = t.ekf_cov[0];
            const p_xy = t.ekf_cov[1];
            const p_yy = t.ekf_cov[2];
            const trace = p_xx + p_yy;
            const diff = p_xx - p_yy;
            const term = Math.sqrt(diff * diff + 4 * p_xy * p_xy);
            const lambda1 = (trace + term) / 2;
            const lambda2 = (trace - term) / 2;
            const semiMajor = Math.sqrt(Math.max(0, lambda1));
            const semiMinor = Math.sqrt(Math.max(0, lambda2));
            const theta = 0.5 * Math.atan2(2 * p_xy, diff);

            // Convert axes from meters to kilometers, scale by 2.448 for 95% confidence, then map to pixels
            const aPixels = ((semiMajor / 1000) * 2.448 / 70) * maxRadius;
            const bPixels = ((semiMinor / 1000) * 2.448 / 70) * maxRadius;

            radarCtx.save();
            radarCtx.strokeStyle = color;
            radarCtx.lineWidth = 1;
            radarCtx.setLineDash([4, 4]);
            radarCtx.beginPath();
            radarCtx.ellipse(pos.x, pos.y, aPixels, bPixels, -theta, 0, Math.PI * 2);
            radarCtx.stroke();
            radarCtx.restore();
        }

        if (isSelected) {
            // Draw square lockbox around selected target
            radarCtx.strokeStyle = "#ffea00";
            radarCtx.lineWidth = 1.5;
            radarCtx.strokeRect(pos.x - 9, pos.y - 9, 18, 18);

            // Crosshair inner dots
            radarCtx.fillStyle = "#ffea00";
            radarCtx.fillRect(pos.x - 1, pos.y - 1, 2, 2);

            // Vector line
            const velScale = 20; // scale factor for visual speed
            const endX = t.pos_enu[0] / 1000 + (t.vel_enu[0] / 1000) * velScale;
            const endY = t.pos_enu[1] / 1000 + (t.vel_enu[1] / 1000) * velScale;
            const endPos = enuToPixel(endX, endY);
            
            radarCtx.strokeStyle = "#ffea00";
            radarCtx.lineWidth = 1;
            radarCtx.beginPath();
            radarCtx.moveTo(pos.x, pos.y);
            radarCtx.lineTo(endPos.x, endPos.y);
            radarCtx.stroke();

            // Highlighted detailed text
            radarCtx.fillStyle = "#00e5ff";
            radarCtx.font = "11px Share Tech Mono";
            radarCtx.fillText(`${t.callsign || 'T' + t.id} [${t.classification || 'UNKNOWN'}]`, pos.x + 12, pos.y - 6);
            radarCtx.fillText(`ALT: ${(t.pos_enu[2]/1000).toFixed(1)}km, SPD: ${t.speed_mps.toFixed(0)}m/s`, pos.x + 12, pos.y + 6);
        } else {
            // Render non-selected target as a clean vector arrowhead pointing in flight direction
            const vx = t.vel_enu[0];
            const vy = t.vel_enu[1];
            const vLen = Math.sqrt(vx*vx + vy*vy);

            radarCtx.fillStyle = color;
            if (vLen > 1.0) {
                const dx = vx / vLen;
                const dy = vy / vLen;
                const px = -dy;
                const py = dx;

                // local polygon vertices
                const tx = t.pos_enu[0]/1000;
                const ty = t.pos_enu[1]/1000;

                const tip = enuToPixel(tx + dx * 0.9, ty + dy * 0.9);
                const left = enuToPixel(tx - dx * 0.5 + px * 0.4, ty - dy * 0.5 + py * 0.4);
                const right = enuToPixel(tx - dx * 0.5 - px * 0.4, ty - dy * 0.5 - py * 0.4);

                radarCtx.beginPath();
                radarCtx.moveTo(tip.x, tip.y);
                radarCtx.lineTo(left.x, left.y);
                radarCtx.lineTo(right.x, right.y);
                radarCtx.closePath();
                radarCtx.fill();
            } else {
                radarCtx.beginPath();
                radarCtx.arc(pos.x, pos.y, 3, 0, Math.PI*2);
                radarCtx.fill();
            }

            // Minimal label
            radarCtx.fillStyle = "rgba(0, 229, 255, 0.7)";
            radarCtx.font = "9px Share Tech Mono";
            radarCtx.fillText(t.callsign || `T${t.id}`, pos.x + 8, pos.y - 4);
        }
    });

    // 5. Draw Sweeper Beam
    const sweepRadius = maxRadius;
    const gradient = radarCtx.createRadialGradient(cx, cy, 0, cx, cy, sweepRadius);
    gradient.addColorStop(0, "rgba(0, 229, 255, 0.15)");
    gradient.addColorStop(1, "rgba(0, 229, 255, 0.0)");

    radarCtx.fillStyle = gradient;
    radarCtx.beginPath();
    radarCtx.moveTo(cx, cy);
    // Draw sector for sweep fading trail
    const arcSegments = 20;
    for (let i = 0; i <= arcSegments; i++) {
        const angle = sweepAngle - (i / arcSegments) * (Math.PI / 6); // 30 deg trail
        const rx = cx + Math.cos(angle) * sweepRadius;
        const ry = cy + Math.sin(angle) * sweepRadius;
        
        // fade gradient relative to trail age
        radarCtx.fillStyle = `rgba(0, 229, 255, ${0.15 * (1 - i / arcSegments)})`;
        radarCtx.beginPath();
        radarCtx.moveTo(cx, cy);
        const nextAngle = sweepAngle - ((i - 1) / arcSegments) * (Math.PI / 6);
        radarCtx.lineTo(cx + Math.cos(angle) * sweepRadius, cy + Math.sin(angle) * sweepRadius);
        radarCtx.lineTo(cx + Math.cos(nextAngle) * sweepRadius, cy + Math.sin(nextAngle) * sweepRadius);
        radarCtx.closePath();
        radarCtx.fill();
    }

    // Glowing front line of the sweep
    radarCtx.strokeStyle = "rgba(0, 229, 255, 0.8)";
    radarCtx.lineWidth = 1.5;
    radarCtx.beginPath();
    radarCtx.moveTo(cx, cy);
    radarCtx.lineTo(cx + Math.cos(sweepAngle) * sweepRadius, cy + Math.sin(sweepAngle) * sweepRadius);
    radarCtx.stroke();

    const prevAngle = sweepAngle;
    sweepAngle = (sweepAngle + sweepSpeed) % (Math.PI * 2);

    // Check sweep crossing for each target
    targets.forEach(t => {
        const pos = enuToPixel(t.pos_enu[0] / 1000, t.pos_enu[1] / 1000);
        let tAngle = normAngle(Math.atan2(pos.y - cy, pos.x - cx));
        if (angleBetween(tAngle, prevAngle, sweepAngle)) {
            // Trigger Doppler Beep
            const tx = t.pos_enu[0];
            const ty = t.pos_enu[1];
            const tz = t.pos_enu[2];
            const vx = t.vel_enu[0];
            const vy = t.vel_enu[1];
            const vz = t.vel_enu[2];
            const range = Math.sqrt(tx*tx + ty*ty + tz*tz);
            const radialVel = range > 0.1 ? (vx * tx + vy * ty + vz * tz) / range : 0;
            const lambda = 299792458.0 / centerFreq;
            const dopplerShift = -radialVel / lambda;
            
            let pitch = 440 + dopplerShift;
            if (isNaN(pitch) || pitch < 200) pitch = 200;
            if (pitch > 2500) pitch = 2500;
            
            playDopplerBeep(pitch);
        }
    });
}

// Canvas Click interaction to select target
radarCanvas.addEventListener("click", (e) => {
    const rect = radarCanvas.getBoundingClientRect();
    const clickX = e.clientX - rect.left;
    const clickY = e.clientY - rect.top;

    const w = radarCanvas.width;
    const h = radarCanvas.height;
    const cx = w / 2;
    const cy = h / 2;
    const maxRadius = Math.min(cx, cy) * 0.95;

    // Convert pixel to ENU
    const pxToEnu = (x, y) => {
        const x_km = ((x - cx) / maxRadius) * 70;
        const y_km = -((y - cy) / maxRadius) * 70;
        return { x: x_km * 1000, y: y_km * 1000 };
    };

    const clickEnu = pxToEnu(clickX, clickY);

    // Find closest target within threshold
    let closestTarget = null;
    let minDistance = 5000; // 5 km threshold

    targets.forEach(t => {
        const dx = t.pos_enu[0] - clickEnu.x;
        const dy = t.pos_enu[1] - clickEnu.y;
        const dist = Math.sqrt(dx*dx + dy*dy);
        if (dist < minDistance) {
            minDistance = dist;
            closestTarget = t;
        }
    });

    if (closestTarget) {
        selectTarget(closestTarget.id);
    } else {
        selectedTargetId = null;
        updateTelemetryTable();
    }
});

// Helper to project 3D ENU coordinate in km to 2D screen coordinate
function project3D(x_km, y_km, z_km, w, h) {
    const cx = w / 2;
    const cy = h / 2;

    // Translate Z to center height around 7.5
    const x1 = x_km;
    const y1 = y_km;
    const z1 = z_km - 7.5;

    // Yaw rotation
    const cosYaw = Math.cos(yaw3d);
    const sinYaw = Math.sin(yaw3d);
    const x2 = x1 * cosYaw - y1 * sinYaw;
    const y2 = x1 * sinYaw + y1 * cosYaw;
    const z2 = z1;

    // Pitch rotation
    const cosPitch = Math.cos(pitch3d);
    const sinPitch = Math.sin(pitch3d);
    const x3 = x2;
    const y3 = y2 * cosPitch - z2 * sinPitch;
    const z3 = y2 * sinPitch + z2 * cosPitch;

    // Scale to fit the 140km width box on screen
    const maxDim = 140;
    const scale = Math.min(w, h) * 0.72 / maxDim;

    const screenX = cx + x3 * scale;
    const screenY = cy - z3 * scale; // screen coordinates are y-down

    return { x: screenX, y: screenY };
}

// Draw line in 3D ENU space
function drawLine3D(ctx, x1, y1, z1, x2, y2, z2, w, h) {
    const p1 = project3D(x1, y1, z1, w, h);
    const p2 = project3D(x2, y2, z2, w, h);
    ctx.beginPath();
    ctx.moveTo(p1.x, p1.y);
    ctx.lineTo(p2.x, p2.y);
    ctx.stroke();
}

// 3D Elevation Profile Paint Loop
function draw3DElevation() {
    requestAnimationFrame(draw3DElevation);

    if (!elevationCanvas) return;
    const w = elevationCanvas.width;
    const h = elevationCanvas.height;
    if (w === 0 || h === 0) return;

    // Persistence/Clear
    elevationCtx.fillStyle = "rgba(3, 9, 20, 0.4)";
    elevationCtx.fillRect(0, 0, w, h);

    // 1. Draw 3D bounding box (Z=0 and Z=15 km)
    elevationCtx.strokeStyle = "rgba(0, 229, 255, 0.08)";
    elevationCtx.lineWidth = 1;
    // Bottom square (ground)
    drawLine3D(elevationCtx, -70, -70, 0, 70, -70, 0, w, h);
    drawLine3D(elevationCtx, 70, -70, 0, 70, 70, 0, w, h);
    drawLine3D(elevationCtx, 70, 70, 0, -70, 70, 0, w, h);
    drawLine3D(elevationCtx, -70, 70, 0, -70, -70, 0, w, h);
    // Top square
    drawLine3D(elevationCtx, -70, -70, 15, 70, -70, 15, w, h);
    drawLine3D(elevationCtx, 70, -70, 15, 70, 70, 15, w, h);
    drawLine3D(elevationCtx, 70, 70, 15, -70, 70, 15, w, h);
    drawLine3D(elevationCtx, -70, 70, 15, -70, -70, 15, w, h);
    // Vertical edges
    drawLine3D(elevationCtx, -70, -70, 0, -70, -70, 15, w, h);
    drawLine3D(elevationCtx, 70, -70, 0, 70, -70, 15, w, h);
    drawLine3D(elevationCtx, 70, 70, 0, 70, 70, 15, w, h);
    drawLine3D(elevationCtx, -70, 70, 0, -70, 70, 15, w, h);

    // 2. Draw ground grid parallel lines
    elevationCtx.strokeStyle = "rgba(0, 229, 255, 0.04)";
    const divisions = 4;
    const step = 140 / divisions;
    for (let i = 1; i < divisions; i++) {
        const val = -70 + i * step;
        drawLine3D(elevationCtx, val, -70, 0, val, 70, 0, w, h);
        drawLine3D(elevationCtx, -70, val, 0, 70, val, 0, w, h);
    }

    // 3. Draw vertical altitude axis and ticks (0-15km)
    elevationCtx.strokeStyle = "rgba(0, 229, 255, 0.2)";
    drawLine3D(elevationCtx, 0, 0, 0, 0, 0, 15, w, h);
    for (let alt = 0; alt <= 15; alt += 3) {
        const tickPos = project3D(0, 0, alt, w, h);
        elevationCtx.fillStyle = "rgba(0, 229, 255, 0.6)";
        elevationCtx.font = "9px Share Tech Mono";
        elevationCtx.textAlign = "left";
        elevationCtx.fillText(`  ${alt} km`, tickPos.x, tickPos.y + 3);
        
        const tickP1 = project3D(-2, 0, alt, w, h);
        const tickP2 = project3D(2, 0, alt, w, h);
        elevationCtx.beginPath();
        elevationCtx.moveTo(tickP1.x, tickP1.y);
        elevationCtx.lineTo(tickP2.x, tickP2.y);
        elevationCtx.stroke();
    }

    // 4. Draw Receiver (Rx)
    const rxPos = project3D(0, 0, 0, w, h);
    elevationCtx.fillStyle = "#00e676";
    elevationCtx.beginPath();
    elevationCtx.arc(rxPos.x, rxPos.y, 4, 0, Math.PI * 2);
    elevationCtx.fill();
    elevationCtx.strokeStyle = "rgba(0, 230, 118, 0.5)";
    elevationCtx.beginPath();
    elevationCtx.arc(rxPos.x, rxPos.y, 8, 0, Math.PI * 2);
    elevationCtx.stroke();

    // 5. Draw Transmitter Towers
    activeTowers.forEach(tow => {
        const tx = tow.pos_enu[0] / 1000;
        const ty = tow.pos_enu[1] / 1000;
        const tPos = project3D(tx, ty, 0, w, h);

        elevationCtx.fillStyle = "#e040fb";
        elevationCtx.beginPath();
        elevationCtx.moveTo(tPos.x, tPos.y - 5);
        elevationCtx.lineTo(tPos.x - 4, tPos.y + 4);
        elevationCtx.lineTo(tPos.x + 4, tPos.y + 4);
        elevationCtx.closePath();
        elevationCtx.fill();

        elevationCtx.fillStyle = "rgba(224, 64, 251, 0.7)";
        elevationCtx.font = "8px Share Tech Mono";
        elevationCtx.textAlign = "center";
        elevationCtx.fillText(tow.name, tPos.x, tPos.y + 12);
    });

    // 6. Draw Targets & Histories
    targets.forEach(t => {
        const isSelected = selectedTargetId === t.id;
        const tx = t.pos_enu[0] / 1000;
        const ty = t.pos_enu[1] / 1000;
        const tz = t.pos_enu[2] / 1000;

        const targetPos = project3D(tx, ty, tz, w, h);
        const groundPos = project3D(tx, ty, 0, w, h);

        let color = "#00e676";
        if (t.state === "Coasting") color = "#ff9100";
        if (t.state === "Suspect") color = "#ffea00";
        if (t.state === "Terminated") color = "#90a4ae";

        // Vertical drop line to ground plane
        elevationCtx.strokeStyle = "rgba(0, 229, 255, 0.3)";
        elevationCtx.lineWidth = 1;
        elevationCtx.setLineDash([2, 2]);
        elevationCtx.beginPath();
        elevationCtx.moveTo(targetPos.x, targetPos.y);
        elevationCtx.lineTo(groundPos.x, groundPos.y);
        elevationCtx.stroke();
        elevationCtx.setLineDash([]);

        // Ground target footprint dot
        elevationCtx.fillStyle = "rgba(0, 229, 255, 0.4)";
        elevationCtx.beginPath();
        elevationCtx.arc(groundPos.x, groundPos.y, 2.5, 0, Math.PI * 2);
        elevationCtx.fill();

        // Render target history gradient line
        const history = targetHistoryCache.get(t.id) || [];
        if (history.length > 1) {
            for (let i = 1; i < history.length; i++) {
                const pt1 = project3D(history[i-1][0], history[i-1][1], history[i-1][2], w, h);
                const pt2 = project3D(history[i][0], history[i][1], history[i][2], w, h);
                const opacity = i / history.length;
                
                let baseColor = [0, 230, 118];
                if (t.state === "Coasting") baseColor = [255, 145, 0];
                if (t.state === "Suspect") baseColor = [255, 234, 0];
                if (t.state === "Terminated") baseColor = [144, 164, 174];

                elevationCtx.strokeStyle = `rgba(${baseColor[0]}, ${baseColor[1]}, ${baseColor[2]}, ${opacity * 0.75})`;
                elevationCtx.lineWidth = isSelected ? 2.0 : 1.0;
                elevationCtx.beginPath();
                elevationCtx.moveTo(pt1.x, pt1.y);
                elevationCtx.lineTo(pt2.x, pt2.y);
                elevationCtx.stroke();
            }
        }

        // Draw marker
        if (isSelected) {
            // 3D wireframe box
            const d = 1.5; // box half-size in km
            const vertices = [];
            for (let dx of [-d, d]) {
                for (let dy of [-d, d]) {
                    for (let dz of [-d, d]) {
                        vertices.push(project3D(tx + dx, ty + dy, tz + dz, w, h));
                    }
                }
            }
            const edges = [
                [0, 1], [0, 2], [0, 4],
                [3, 1], [3, 2], [3, 7],
                [5, 1], [5, 4], [5, 7],
                [6, 2], [6, 4], [6, 7]
            ];
            elevationCtx.strokeStyle = "#ffea00"; // yellow lock box
            elevationCtx.lineWidth = 1.5;
            edges.forEach(([idx1, idx2]) => {
                elevationCtx.beginPath();
                elevationCtx.moveTo(vertices[idx1].x, vertices[idx1].y);
                elevationCtx.lineTo(vertices[idx2].x, vertices[idx2].y);
                elevationCtx.stroke();
            });

            // Target Text details
            elevationCtx.fillStyle = "#ffea00";
            elevationCtx.font = "10px Share Tech Mono";
            elevationCtx.textAlign = "left";
            elevationCtx.fillText(`${t.callsign || 'T' + t.id} ALT: ${tz.toFixed(2)}km`, targetPos.x + 12, targetPos.y - 4);
        } else {
            // Normal sphere marker
            elevationCtx.fillStyle = color;
            elevationCtx.beginPath();
            elevationCtx.arc(targetPos.x, targetPos.y, 4, 0, Math.PI * 2);
            elevationCtx.fill();

            // Label
            elevationCtx.fillStyle = "rgba(178, 235, 242, 0.7)";
            elevationCtx.font = "9px Share Tech Mono";
            elevationCtx.textAlign = "left";
            elevationCtx.fillText(t.callsign || `T${t.id}`, targetPos.x + 6, targetPos.y - 2);
        }
    });
}

// JEM Spectrum Paint Loop
function shiftAndDrawJemWaterfall(bins) {
    if (!jemCanvas) return;
    const w = jemCanvas.width;
    const h = jemCanvas.height;
    if (w === 0 || h === 0) return;

    // Initialize offscreen canvas if needed
    if (!jemOffscreenCanvas || jemOffscreenCanvas.width !== w || jemOffscreenCanvas.height !== h) {
        jemOffscreenCanvas = document.createElement("canvas");
        jemOffscreenCanvas.width = w;
        jemOffscreenCanvas.height = h;
        jemOffscreenCtx = jemOffscreenCanvas.getContext("2d");
        jemOffscreenCtx.fillStyle = "rgba(3, 9, 20, 1.0)";
        jemOffscreenCtx.fillRect(0, 0, w, h);
    }

    const scrollAmount = 2; // shift left by 2px

    // Shift offscreen canvas left
    jemOffscreenCtx.drawImage(jemOffscreenCanvas, scrollAmount, 0, w - scrollAmount, h, 0, 0, w - scrollAmount, h);

    // Clear rightmost column and fill with solid background
    const colX = w - scrollAmount;
    const colWidth = scrollAmount;
    jemOffscreenCtx.clearRect(colX, 0, colWidth, h);
    jemOffscreenCtx.fillStyle = "rgba(3, 9, 20, 1.0)";
    jemOffscreenCtx.fillRect(colX, 0, colWidth, h);

    const numBins = bins.length;

    // Find max value in this bin array
    let maxVal = 0.01;
    for (let i = 0; i < numBins; i++) {
        if (bins[i] > maxVal) {
            maxVal = bins[i];
        }
    }

    const binHeight = h / numBins;
    for (let i = 0; i < numBins; i++) {
        // Map so index 0 is at bottom (i=0 -> y=h) and index numBins-1 is at top (i=numBins-1 -> y=0)
        const y = h - (i / numBins) * h;
        const normVal = bins[i] / maxVal;

        // Phosphor gradient: green to cyan
        const norm = Math.max(0, Math.min(1, normVal));
        const g = Math.floor(norm * 229);
        const b = Math.floor(norm * 255);
        jemOffscreenCtx.fillStyle = `rgba(0, ${g}, ${b}, ${norm})`;
        
        jemOffscreenCtx.fillRect(colX, y - binHeight, colWidth, binHeight + 0.5);
    }
}

// JEM Spectrum Paint Loop
function drawJEMSpectrum() {
    requestAnimationFrame(drawJEMSpectrum);

    if (!jemCanvas) return;
    const w = jemCanvas.width;
    const h = jemCanvas.height;
    if (w === 0 || h === 0) return;

    if (selectedTargetId === null) {
        jemCtx.fillStyle = "rgba(3, 9, 20, 0.4)";
        jemCtx.fillRect(0, 0, w, h);
        jemCtx.fillStyle = "rgba(178, 235, 242, 0.4)";
        jemCtx.font = "12px Share Tech Mono";
        jemCtx.textAlign = "center";
        jemCtx.textBaseline = "middle";
        jemCtx.fillText("SELECT A TARGET TO VIEW JEM SPECTRUM", w / 2, h / 2);
        return;
    }

    const t = targets.find(target => target.id === selectedTargetId);
    if (!t || !t.jem_fft_mag || t.jem_fft_mag.length === 0) {
        jemCtx.fillStyle = "rgba(3, 9, 20, 0.4)";
        jemCtx.fillRect(0, 0, w, h);
        jemCtx.fillStyle = "rgba(178, 235, 242, 0.4)";
        jemCtx.font = "12px Share Tech Mono";
        jemCtx.textAlign = "center";
        jemCtx.textBaseline = "middle";
        const selT = targets.find(target => target.id === selectedTargetId);
        const ident = selT ? (selT.callsign || 'T' + selectedTargetId) : ('T' + selectedTargetId);
        jemCtx.fillText("NO JEM SPECTRUM TELEMETRY FOR TARGET " + ident, w / 2, h / 2);
        return;
    }

    if (jemViewMode === "SPECTROGRAM") {
        if (jemOffscreenCanvas) {
            jemCtx.drawImage(jemOffscreenCanvas, 0, 0);
        } else {
            jemCtx.fillStyle = "rgba(3, 9, 20, 1.0)";
            jemCtx.fillRect(0, 0, w, h);
        }

        const padding = 15;
        jemCtx.fillStyle = "rgba(0, 229, 255, 0.8)";
        jemCtx.font = "9px Share Tech Mono";
        jemCtx.textAlign = "left";
        jemCtx.textBaseline = "top";
        jemCtx.fillText("MODE: MICRO-DOPPLER SPECTROGRAM", padding + 10, padding + 10);
        jemCtx.fillText(`TARGET: ${t.callsign || 'T' + t.id} [${t.classification || 'AIRCRAFT'}]`, padding + 10, padding + 22);
        if (t.jem_frequency_hz !== undefined && t.jem_frequency_hz !== null) {
            jemCtx.fillText(`BPF: ${t.jem_frequency_hz.toFixed(1)} Hz`, padding + 10, padding + 34);
        }
        
        if (currentJemTemplate && jemTemplates[currentJemTemplate] && currentJemTemplate !== 'none') {
            const tmpl = jemTemplates[currentJemTemplate];
            jemCtx.strokeStyle = "rgba(255, 0, 255, 0.5)";
            jemCtx.lineWidth = 1.5;
            jemCtx.setLineDash([6, 4]);
            
            for (let hz of tmpl.lines) {
                const hzNorm = (hz + 500) / 1000.0; 
                const py = h - hzNorm * h;
                jemCtx.beginPath();
                jemCtx.moveTo(0, py);
                jemCtx.lineTo(w, py);
                jemCtx.stroke();
            }
            jemCtx.setLineDash([]);
            
            jemCtx.fillStyle = "rgba(255, 0, 255, 0.9)";
            jemCtx.fillText(`OVERLAY: ${tmpl.name}`, padding + 10, padding + 46);
            if (tmpl.desc) {
                jemCtx.fillText(`${tmpl.desc}`, padding + 10, padding + 58);
            }
        }
    } else {
        jemCtx.fillStyle = "rgba(3, 9, 20, 0.4)";
        jemCtx.fillRect(0, 0, w, h);

        const bins = t.jem_fft_mag;
        const numBins = bins.length;
        const padding = 15;

        let maxVal = 0.01;
        for (let i = 0; i < numBins; i++) {
            if (bins[i] > maxVal) {
                maxVal = bins[i];
            }
        }

        const points = [];
        const binWidth = (w - 2 * padding) / (numBins - 1);
        for (let i = 0; i < numBins; i++) {
            const x = padding + i * binWidth;
            const y = h - padding - (bins[i] / maxVal) * (h - 2 * padding);
            points.push({ x, y });
        }

        jemCtx.strokeStyle = "#00e5ff";
        jemCtx.lineWidth = 1.5;
        jemCtx.beginPath();
        jemCtx.moveTo(points[0].x, points[0].y);
        for (let i = 1; i < numBins; i++) {
            jemCtx.lineTo(points[i].x, points[i].y);
        }
        jemCtx.stroke();

        const fillGrad = jemCtx.createLinearGradient(0, padding, 0, h - padding);
        fillGrad.addColorStop(0, "rgba(0, 229, 255, 0.25)");
        fillGrad.addColorStop(1, "rgba(0, 229, 255, 0.0)");
        jemCtx.fillStyle = fillGrad;
        jemCtx.beginPath();
        jemCtx.moveTo(points[0].x, h - padding);
        for (let i = 0; i < numBins; i++) {
            jemCtx.lineTo(points[i].x, points[i].y);
        }
        jemCtx.lineTo(points[numBins - 1].x, h - padding);
        jemCtx.closePath();
        jemCtx.fill();

        jemCtx.strokeStyle = "rgba(0, 229, 255, 0.1)";
        jemCtx.lineWidth = 1;
        jemCtx.beginPath();
        jemCtx.moveTo(padding, padding);
        jemCtx.lineTo(padding, h - padding);
        jemCtx.lineTo(w - padding, h - padding);
        jemCtx.stroke();

        jemCtx.fillStyle = "rgba(0, 229, 255, 0.6)";
        jemCtx.font = "9px Share Tech Mono";
        jemCtx.textAlign = "left";
        jemCtx.textBaseline = "top";
        jemCtx.fillText(`PEAK: ${maxVal.toFixed(1)} dB`, padding + 10, padding + 10);
        jemCtx.fillText(`TARGET: ${t.callsign || 'T' + t.id} [${t.classification || 'AIRCRAFT'}]`, padding + 10, padding + 22);
        if (t.jem_frequency_hz !== undefined && t.jem_frequency_hz !== null) {
            jemCtx.fillText(`BPF: ${t.jem_frequency_hz.toFixed(1)} Hz`, padding + 10, padding + 34);
        }

        if (currentJemTemplate && jemTemplates[currentJemTemplate] && currentJemTemplate !== 'none') {
            const tmpl = jemTemplates[currentJemTemplate];
            jemCtx.strokeStyle = "rgba(255, 0, 255, 0.4)";
            jemCtx.lineWidth = 1;
            jemCtx.setLineDash([4, 4]);
            
            for (let hz of tmpl.lines) {
                const hzNorm = (hz + 500) / 1000.0; 
                const px = padding + hzNorm * (w - 2 * padding);
                jemCtx.beginPath();
                jemCtx.moveTo(px, padding);
                jemCtx.lineTo(px, h - padding);
                jemCtx.stroke();
            }

            // Draw envelope outline if present
            if (tmpl.envelope) {
                jemCtx.strokeStyle = "rgba(255, 0, 255, 0.2)";
                jemCtx.lineWidth = 2;
                jemCtx.setLineDash([]);
                const spanHz = tmpl.envelope.span;
                const minHz = -spanHz/2;
                const maxHz = spanHz/2;
                const minPx = padding + ((minHz + 500) / 1000.0) * (w - 2 * padding);
                const maxPx = padding + ((maxHz + 500) / 1000.0) * (w - 2 * padding);
                
                jemCtx.beginPath();
                if (tmpl.envelope.shape === "steep") {
                    jemCtx.moveTo(minPx - 10, h - padding);
                    jemCtx.lineTo(minPx, padding + 20);
                    jemCtx.lineTo(maxPx, padding + 20);
                    jemCtx.lineTo(maxPx + 10, h - padding);
                } else if (tmpl.envelope.shape === "wide" || tmpl.envelope.shape === "turbofan") {
                    jemCtx.moveTo(minPx - 50, h - padding);
                    jemCtx.quadraticCurveTo(w/2, padding - 20, maxPx + 50, h - padding);
                } else {
                    jemCtx.moveTo(minPx, h - padding);
                    jemCtx.quadraticCurveTo(w/2, padding, maxPx, h - padding);
                }
                jemCtx.stroke();
            }

            jemCtx.setLineDash([]);
            jemCtx.fillStyle = "rgba(255, 0, 255, 0.9)";
            jemCtx.fillText(`OVERLAY: ${tmpl.name}`, padding + 10, padding + 46);
            if (tmpl.desc) {
                jemCtx.fillText(`${tmpl.desc}`, padding + 10, padding + 58);
            }
        }
    }
}

// 3D Drag Rotation Event Listeners
if (elevationCanvas) {
    elevationCanvas.addEventListener("mousedown", (e) => {
        isDragging3d = true;
        previousMousePosition = { x: e.clientX, y: e.clientY };
    });

    window.addEventListener("mousemove", (e) => {
        if (isDragging3d) {
            const deltaX = e.clientX - previousMousePosition.x;
            const deltaY = e.clientY - previousMousePosition.y;

            yaw3d += deltaX * 0.007;
            pitch3d += deltaY * 0.007;

            // clamp pitch to avoid flipping upside down
            pitch3d = Math.max(-Math.PI / 2 + 0.05, Math.min(Math.PI / 2 - 0.05, pitch3d));

            previousMousePosition = { x: e.clientX, y: e.clientY };
        }
    });

    window.addEventListener("mouseup", () => {
        isDragging3d = false;
    });
}
// Fullscreen Zoom Buttons Click Handler
document.querySelectorAll(".zoom-btn").forEach(btn => {
    btn.addEventListener("click", (e) => {
        const targetClass = btn.getAttribute("data-target");
        const panel = document.querySelector("." + targetClass);
        if (panel) {
            const isFull = panel.classList.toggle("fullscreen-zoom");
            btn.innerText = isFull ? "EXIT FULLSCREEN" : "FULLSCREEN";
            resizeCanvases();
        }
        e.stopPropagation();
    });
});

// Two-Way Control Panel inputs listeners
const dspThresholdInput = document.getElementById("dsp-threshold");
const dspThresholdVal = document.getElementById("dsp-threshold-val");
const ellipseToggleBtn = document.getElementById("ellipse-toggle-btn");

if (dspThresholdInput) {
    dspThresholdInput.addEventListener("input", (e) => {
        const val = parseFloat(e.target.value);
        if (dspThresholdVal) {
            dspThresholdVal.innerText = `${val.toFixed(1)} dB`;
        }
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({
                command: "set_threshold",
                value: val
            }));
        }
    });
}

const sdrGainInputListener = document.getElementById("sdr-gain");
const sdrGainValListener = document.getElementById("sdr-gain-val");
const sdrOffsetInputListener = document.getElementById("sdr-offset");
const sdrDcBlockInputListener = document.getElementById("sdr-dc-block");

if (sdrGainInputListener) {
    sdrGainInputListener.addEventListener("input", (e) => {
        const val = parseFloat(e.target.value);
        if (sdrGainValListener) sdrGainValListener.innerText = `${val.toFixed(0)} dB`;
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ command: "set_gain", value: val }));
        }
    });
}

const sdrAgcInputListener = document.getElementById("sdr-agc");
if (sdrAgcInputListener) {
    sdrAgcInputListener.addEventListener("change", (e) => {
        const val = e.target.checked;
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ command: "set_agc", enabled: val }));
        }
    });
}

if (sdrOffsetInputListener) {
    sdrOffsetInputListener.addEventListener("change", (e) => {
        const val = parseFloat(e.target.value);
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ command: "set_offset", value: val }));
        }
    });
}

if (sdrDcBlockInputListener) {
    sdrDcBlockInputListener.addEventListener("change", (e) => {
        const val = e.target.checked;
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ command: "set_dc_block", value: val }));
        }
    });
}

const sdrOneBitInputListener = document.getElementById("sdr-one-bit");
if (sdrOneBitInputListener) {
    sdrOneBitInputListener.addEventListener("change", (e) => {
        const val = e.target.checked;
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ command: "set_one_bit_mode", value: val }));
        }
    });
}

const unconfirmedToggleListener = document.getElementById("unconfirmed-toggle");
if (unconfirmedToggleListener) {
    unconfirmedToggleListener.addEventListener("change", (e) => {
        const val = e.target.checked;
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ command: "set_show_unconfirmed", value: val }));
        }
    });
}

const shakeToggleListener = document.getElementById("shake-toggle");
if (shakeToggleListener) {
    shakeToggleListener.addEventListener("change", (e) => {
        const val = e.target.checked;
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ command: "set_screen_shake", value: val }));
        }
    });
}

if (ellipseToggleBtn) {
    ellipseToggleBtn.addEventListener("click", () => {
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({
                command: "toggle_ellipse_mode"
            }));
        }
    });
}

// Aligner canvas click listener
if (alignerCanvas) {
    alignerCanvas.addEventListener("click", (e) => {
        const rect = alignerCanvas.getBoundingClientRect();
        const cx = alignerCanvas.width / 2;
        const cy = alignerCanvas.height / 2;
        const clickX = e.clientX - rect.left - cx;
        const clickY = e.clientY - rect.top - cy;
        
        let phi = Math.atan2(clickY, clickX);
        let heading = (phi * 180) / Math.PI + 90;
        if (heading < 0) heading += 360;
        heading = Math.round(heading % 360);
        
        antennaHeading = heading;
        const antennaHeadingSlider = document.getElementById("antenna-heading");
        const antennaHeadingVal = document.getElementById("antenna-heading-val");
        if (antennaHeadingSlider) {
            antennaHeadingSlider.value = heading;
        }
        if (antennaHeadingVal) {
            antennaHeadingVal.innerText = `${heading}°`;
        }
        
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({
                command: "CalibrateAntenna",
                angle: heading
            }));
        }
    });
}

// Antenna Heading slider listener
const antennaHeadingSlider = document.getElementById("antenna-heading");
const antennaHeadingVal = document.getElementById("antenna-heading-val");
if (antennaHeadingSlider) {
    antennaHeadingSlider.addEventListener("input", (e) => {
        const angle = parseInt(e.target.value);
        if (antennaHeadingVal) {
            antennaHeadingVal.innerText = `${angle}°`;
        }
        antennaHeading = angle;
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({
                command: "CalibrateAntenna",
                angle: angle
            }));
        }
    });
}

// JEM dual-view toggle listener
const jemToggleBtn = document.getElementById("jem-toggle-btn");
if (jemToggleBtn) {
    jemToggleBtn.addEventListener("click", () => {
        jemViewMode = (jemViewMode === "FFT") ? "SPECTROGRAM" : "FFT";
        jemToggleBtn.textContent = (jemViewMode === "FFT") ? "SWITCH TO SPECTROGRAM" : "SWITCH TO FFT";
        if (jemCanvas && jemOffscreenCtx) {
            jemOffscreenCtx.fillStyle = "rgba(3, 9, 20, 1.0)";
            jemOffscreenCtx.fillRect(0, 0, jemCanvas.width, jemCanvas.height);
        }
    });

    const jemTemplateSelect = document.getElementById("jem-template-select");
    if (jemTemplateSelect) {
        jemTemplateSelect.addEventListener("change", (e) => {
            currentJemTemplate = e.target.value;
            // Clear waterfall canvas on switch so boundaries render cleanly
            if (jemCanvas && jemOffscreenCtx) {
                jemOffscreenCtx.fillStyle = "rgba(3, 9, 20, 1.0)";
                jemOffscreenCtx.fillRect(0, 0, jemCanvas.width, jemCanvas.height);
            }
        });
    }
}

// Antenna Alignment Scope Paint Loop
function drawAntennaAligner() {
    requestAnimationFrame(drawAntennaAligner);

    if (!alignerCanvas || !alignerCtx) return;
    const w = alignerCanvas.width;
    const h = alignerCanvas.height;
    if (w === 0 || h === 0) return;

    // Clear background
    alignerCtx.fillStyle = "rgba(3, 9, 20, 0.4)";
    alignerCtx.fillRect(0, 0, w, h);

    const cx = w / 2;
    const cy = h / 2;
    const maxR = Math.min(cx, cy) * 0.8;

    // 1. Draw concentric compass rings
    alignerCtx.strokeStyle = "rgba(0, 229, 255, 0.15)";
    alignerCtx.lineWidth = 1;
    for (let r = 0.25; r <= 1.0; r += 0.25) {
        alignerCtx.beginPath();
        alignerCtx.arc(cx, cy, maxR * r, 0, Math.PI * 2);
        alignerCtx.stroke();
    }

    // 2. Draw radial lines every 30 degrees
    alignerCtx.strokeStyle = "rgba(0, 229, 255, 0.08)";
    for (let angle = 0; angle < 360; angle += 30) {
        const rad = (angle * Math.PI) / 180;
        alignerCtx.beginPath();
        alignerCtx.moveTo(cx, cy);
        alignerCtx.lineTo(cx + maxR * Math.cos(rad), cy + maxR * Math.sin(rad));
        alignerCtx.stroke();
    }

    // 3. Draw Cardinal Direction indicators
    alignerCtx.fillStyle = "rgba(0, 229, 255, 0.7)";
    alignerCtx.font = "11px Orbitron, Share Tech Mono, sans-serif";
    alignerCtx.textAlign = "center";
    alignerCtx.textBaseline = "middle";
    alignerCtx.fillText("N", cx, cy - maxR - 12);
    alignerCtx.fillText("S", cx, cy + maxR + 12);
    alignerCtx.fillText("E", cx + maxR + 12, cy);
    alignerCtx.fillText("W", cx - maxR - 12, cy);

    // 4. Draw Transmitter lines and dots (using calculated bearings)
    activeTowers.forEach(tow => {
        if (!tow.pos_enu || tow.pos_enu.length < 2) return;
        const x = tow.pos_enu[0];
        const y = tow.pos_enu[1];
        let bearingDeg = Math.atan2(x, y) * 180 / Math.PI;
        if (bearingDeg < 0) bearingDeg += 360;
        const bearingRad = ((bearingDeg - 90) * Math.PI) / 180;

        alignerCtx.strokeStyle = "rgba(224, 64, 251, 0.6)"; // Pink dashed line
        alignerCtx.lineWidth = 1.5;
        alignerCtx.setLineDash([2, 4]);
        alignerCtx.beginPath();
        alignerCtx.moveTo(cx, cy);
        alignerCtx.lineTo(cx + maxR * Math.cos(bearingRad), cy + maxR * Math.sin(bearingRad));
        alignerCtx.stroke();
        alignerCtx.setLineDash([]);

        // Draw Transmitter dot
        alignerCtx.fillStyle = "#e040fb";
        alignerCtx.beginPath();
        alignerCtx.arc(cx + maxR * Math.cos(bearingRad), cy + maxR * Math.sin(bearingRad), 4, 0, Math.PI * 2);
        alignerCtx.fill();

        // Draw callsign label
        alignerCtx.fillStyle = "#e040fb";
        alignerCtx.font = "9px Share Tech Mono";
        alignerCtx.textAlign = Math.cos(bearingRad) >= 0 ? "left" : "right";
        alignerCtx.textBaseline = "middle";
        const offsetDist = 8;
        const textX = cx + (maxR + offsetDist) * Math.cos(bearingRad);
        const textY = cy + (maxR + offsetDist) * Math.sin(bearingRad);
        alignerCtx.fillText(tow.name || "TX", textX, textY);
    });

    // 5. Draw steerable receiver pattern (green cardioid)
    const headingRad = ((antennaHeading - 90) * Math.PI) / 180;
    alignerCtx.strokeStyle = "rgba(0, 230, 118, 0.8)";
    alignerCtx.fillStyle = "rgba(0, 230, 118, 0.15)";
    alignerCtx.lineWidth = 2;
    alignerCtx.beginPath();
    for (let angle = 0; angle <= 360; angle += 5) {
        const phi = (angle * Math.PI) / 180;
        // Cardioid pattern: R(phi) = maxR * (0.15 + 0.85 * (1 + Math.cos(phi - headingRad)) / 2)
        const r = maxR * (0.15 + 0.85 * (1 + Math.cos(phi - headingRad)) / 2);
        const x = cx + r * Math.cos(phi);
        const y = cy + r * Math.sin(phi);
        if (angle === 0) {
            alignerCtx.moveTo(x, y);
        } else {
            alignerCtx.lineTo(x, y);
        }
    }
    alignerCtx.closePath();
    alignerCtx.stroke();
    alignerCtx.fill();

    // 6. Draw null orange pointer (opposite to heading)
    const nullAngleDeg = (antennaHeading + 180) % 360;
    const nullRad = ((nullAngleDeg - 90) * Math.PI) / 180;
    alignerCtx.strokeStyle = "#ff9100";
    alignerCtx.lineWidth = 2;
    alignerCtx.setLineDash([4, 4]);
    alignerCtx.beginPath();
    alignerCtx.moveTo(cx, cy);
    alignerCtx.lineTo(cx + maxR * Math.cos(nullRad), cy + maxR * Math.sin(nullRad));
    alignerCtx.stroke();
    alignerCtx.setLineDash([]);

    // Draw NULL text
    alignerCtx.fillStyle = "#ff9100";
    alignerCtx.font = "9px Share Tech Mono";
    alignerCtx.textAlign = "center";
    alignerCtx.textBaseline = "middle";
    const labelR = maxR * 1.15;
    alignerCtx.fillText("NULL", cx + labelR * Math.cos(nullRad), cy + labelR * Math.sin(nullRad));
}

// Initialization
resizeCanvases();
initConnection();

// Global states
let centerFreq = 90.9e6;
const announcedLocks = new Set();
const announcedMeteors = new Set();

// Math helpers
function normAngle(angle) {
    while (angle < 0) angle += Math.PI * 2;
    while (angle >= Math.PI * 2) angle -= Math.PI * 2;
    return angle;
}

function angleBetween(a, a1, a2) {
    let diff1 = normAngle(a - a1);
    let diff2 = normAngle(a2 - a1);
    return diff1 <= diff2;
}

// Audio Engine Setup
let audioCtx = null;
let ambientOsc = null;
let ambientGain = null;
let masterCompressor = null;

function initAudio() {
    if (audioCtx) return;
    try {
        const AudioContextClass = window.AudioContext || window.webkitAudioContext;
        audioCtx = new AudioContextClass();
        
        // Dynamics Compressor for Master limiting
        masterCompressor = audioCtx.createDynamicsCompressor();
        masterCompressor.threshold.setValueAtTime(-12, audioCtx.currentTime); // dB
        masterCompressor.knee.setValueAtTime(30, audioCtx.currentTime);       // dB
        masterCompressor.ratio.setValueAtTime(12, audioCtx.currentTime);       // compression ratio
        masterCompressor.attack.setValueAtTime(0.003, audioCtx.currentTime);  // 3ms attack
        masterCompressor.release.setValueAtTime(0.08, audioCtx.currentTime);   // 80ms release
        masterCompressor.connect(audioCtx.destination);

        // 55 Hz ambient hum oscillator
        ambientOsc = audioCtx.createOscillator();
        ambientOsc.type = "sine";
        ambientOsc.frequency.setValueAtTime(55, audioCtx.currentTime);
        
        // 0.15 Hz LFO modulation
        const lfo = audioCtx.createOscillator();
        lfo.type = "sine";
        lfo.frequency.setValueAtTime(0.15, audioCtx.currentTime);
        
        const lfoGain = audioCtx.createGain();
        lfoGain.gain.setValueAtTime(8, audioCtx.currentTime); // modulate pitch by +/- 8 Hz
        
        ambientGain = audioCtx.createGain();
        ambientGain.gain.setValueAtTime(0.05, audioCtx.currentTime); // low volume
        
        lfo.connect(lfoGain);
        lfoGain.connect(ambientOsc.frequency);
        
        ambientOsc.connect(ambientGain);
        ambientGain.connect(masterCompressor);
        
        lfo.start();
        ambientOsc.start();
        
        console.log("Web Audio Context initialized.");
    } catch (e) {
        console.error("Failed to initialize audio context:", e);
    }
}

window.addEventListener("click", () => {
    initAudio();
}, { once: true });
window.addEventListener("keydown", () => {
    initAudio();
}, { once: true });

function resumeAudio() {
    if (audioCtx && audioCtx.state === "suspended") {
        audioCtx.resume();
    }
}
window.addEventListener("click", resumeAudio);
window.addEventListener("keydown", resumeAudio);

let nextPlayTime = 0;

function playAudioChunk(float32Array) {
    if (!audioCtx) return;
    if (audioCtx.state === "suspended") return;
    const sampleRate = 8000; // 8 kHz for Ghost Mic Acoustic PCM
    try {
        const buffer = audioCtx.createBuffer(1, float32Array.length, sampleRate);
        const channelData = buffer.getChannelData(0);
        channelData.set(float32Array);

        const source = audioCtx.createBufferSource();
        source.buffer = buffer;
        source.connect(masterCompressor || audioCtx.destination);

        const now = audioCtx.currentTime;
        if (nextPlayTime < now || nextPlayTime - now > 0.5) {
            nextPlayTime = now;
        }
        source.start(nextPlayTime);
        nextPlayTime += buffer.duration;
    } catch (e) {
        console.error("Error playing audio chunk:", e);
    }
}

function playDopplerBeep(freq) {
    if (!audioCtx || audioCtx.state === "suspended") return;
    try {
        const osc = audioCtx.createOscillator();
        const gainNode = audioCtx.createGain();
        
        osc.type = "sine";
        osc.frequency.setValueAtTime(freq, audioCtx.currentTime);
        
        // Fast attack (15ms), exponential decay (300ms)
        gainNode.gain.setValueAtTime(0.0, audioCtx.currentTime);
        gainNode.gain.linearRampToValueAtTime(0.1, audioCtx.currentTime + 0.015);
        gainNode.gain.exponentialRampToValueAtTime(0.001, audioCtx.currentTime + 0.3);
        
        osc.connect(gainNode);
        gainNode.connect(masterCompressor || audioCtx.destination);
        
        osc.onended = () => {
            osc.disconnect();
            gainNode.disconnect();
        };
        
        osc.start();
        osc.stop(audioCtx.currentTime + 0.45);
    } catch (e) {
        console.error("Failed to play Doppler beep:", e);
    }
}

// Speech Voice Alert Setup
let speechQueue = [];
let isSpeaking = false;

function processSpeechQueue() {
    if (speechQueue.length === 0 || isSpeaking) return;
    const phrase = speechQueue.shift();
    isSpeaking = true;
    const utterance = new SpeechSynthesisUtterance(phrase);
    utterance.volume = 0.8;
    utterance.rate = 1.05;
    utterance.pitch = 0.9;
    const voices = window.speechSynthesis.getVoices();
    const voice = voices.find(v => v.lang.startsWith("en"));
    if (voice) utterance.voice = voice;
    utterance.onend = () => {
        isSpeaking = false;
        processSpeechQueue();
    };
    utterance.onerror = () => {
        isSpeaking = false;
        processSpeechQueue();
    };
    window.speechSynthesis.speak(utterance);
}

function speakVoiceAlert(phrase) {
    // Voice announcements disabled as requested by user
    return;
}

// Hacker Terminal UI logic
function printToTerminal(text, className = "resp-ok") {
    const termOutput = document.getElementById("terminal-output");
    if (!termOutput) return;
    const entry = document.createElement("div");
    entry.className = `terminal-line ${className}`;
    entry.innerText = text;
    termOutput.appendChild(entry);
    termOutput.scrollTop = termOutput.scrollHeight;
}

function handleTerminalResponse(data) {
    if (data.error) {
        printToTerminal(`ERROR: ${data.error}`, "resp-err");
    } else if (data.status) {
        printToTerminal(`STATUS: ${data.status}`);
    } else if (data.system_status) {
        printToTerminal(`SYSTEM STATUS: ${data.system_status.toUpperCase()}`);
    } else if (data.frequencies) {
        printToTerminal(`SCAN FREQS: ${data.frequencies.join(" MHz, ")} MHz`);
    } else if (data.jamming) {
        printToTerminal(`JAMMING: ${data.jamming.toUpperCase()}`);
    } else if (data.spoofing) {
        printToTerminal(`SPOOF DECOY INJECTED: Target ID ${data.spoofing}`);
    } else if (data.logs) {
        printToTerminal("=== DUMPING SYSTEM LOGS ===");
        data.logs.forEach(log => {
            printToTerminal(log);
        });
    } else {
        printToTerminal(JSON.stringify(data));
    }
}

function initHackerTerminal() {
    const termInput = document.getElementById("terminal-input");
    if (!termInput) return;

    termInput.addEventListener("keydown", (e) => {
        if (e.key === "Enter") {
            const cmd = termInput.value.trim();
            termInput.value = "";
            if (!cmd) return;

            printToTerminal(`passiveradar_hacker> ${cmd}`, "cmd-echo");
            processHackerCommand(cmd);
        }
    });
}

function processHackerCommand(cmd) {
    const parts = cmd.split(/\s+/);
    const primary = parts[0].toLowerCase();

    if (primary === "help") {
        printToTerminal("Available commands:");
        printToTerminal("  help             - Show this help list");
        printToTerminal("  clear            - Clear the terminal console");
        printToTerminal("  logs             - Dump recent event log messages");
        printToTerminal("  sysinfo          - Fetch DSP backend system state");
        printToTerminal("  scan             - Scan for active tower carrier frequencies");
        printToTerminal("  jam              - Toggle simulated radar jamming");
        printToTerminal("  spoof [id]       - Inject a simulated decoy aircraft target");
        return;
    }

    if (primary === "clear") {
        const termOutput = document.getElementById("terminal-output");
        if (termOutput) termOutput.innerHTML = "";
        return;
    }

    if (primary === "logs") {
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({
                command: "hacker_cmd",
                payload: "logs"
            }));
        } else {
            printToTerminal("ERROR: Backend connection offline.", "resp-err");
        }
        return;
    }

    // Forward to WebSocket backend
    if (ws && ws.readyState === WebSocket.OPEN) {
        let payload = cmd;
        if (primary === "spoof" && parts.length > 1) {
            const targetId = parseInt(parts[1]);
            if (!isNaN(targetId)) {
                const extraArgs = parts.slice(2).join(" ");
                payload = `spoof --id ${targetId}${extraArgs ? " " + extraArgs : ""}`;
            }
        }
        ws.send(JSON.stringify({
            command: "hacker_cmd",
            payload: payload
        }));
    } else {
        printToTerminal("ERROR: Backend connection offline.", "resp-err");
    }
}

// Theme vector canvas helpers
function themeColor(str) {
    if (document.body.classList.contains("green-glow-mode")) {
        return str.replace(/0,\s*229,\s*255/g, "0, 230, 118").replace(/#00e5ff/g, "#00e676");
    }
    return str;
}

// Apply prototype interceptor overrides globally for canvas rendering colors
(function() {
    const originalStrokeStyle = Object.getOwnPropertyDescriptor(CanvasRenderingContext2D.prototype, "strokeStyle");
    const originalFillStyle = Object.getOwnPropertyDescriptor(CanvasRenderingContext2D.prototype, "fillStyle");

    Object.defineProperty(CanvasRenderingContext2D.prototype, "strokeStyle", {
        set: function(val) {
            if (typeof val === "string") {
                val = themeColor(val);
            }
            originalStrokeStyle.set.call(this, val);
        },
        get: function() {
            return originalStrokeStyle.get.call(this);
        }
    });

    Object.defineProperty(CanvasRenderingContext2D.prototype, "fillStyle", {
        set: function(val) {
            if (typeof val === "string") {
                val = themeColor(val);
            }
            originalFillStyle.set.call(this, val);
        },
        get: function() {
            return originalFillStyle.get.call(this);
        }
    });

    const originalAddColorStop = CanvasGradient.prototype.addColorStop;
    CanvasGradient.prototype.addColorStop = function(offset, color) {
        if (typeof color === "string") {
            color = themeColor(color);
        }
        originalAddColorStop.call(this, offset, color);
    };
})();

// Wire up checkboxes
const crtToggle = document.getElementById("crt-toggle");
const glowToggle = document.getElementById("glow-toggle");

if (crtToggle) {
    crtToggle.addEventListener("change", (e) => {
        if (e.target.checked) {
            document.body.classList.add("crt-active");
        } else {
            document.body.classList.remove("crt-active");
        }
    });
}

if (glowToggle) {
    glowToggle.addEventListener("change", (e) => {
        if (e.target.checked) {
            document.body.classList.add("green-glow-mode");
        } else {
            document.body.classList.remove("green-glow-mode");
        }
    });
}

initHackerTerminal();
drawRadarScope();
draw3DElevation();
drawJEMSpectrum();
drawAntennaAligner();

// Phase 3 & 4: Cepstrum Scope draw loop
function drawCepstrumScope() {
    requestAnimationFrame(drawCepstrumScope);

    if (!cepstrumCanvas || !cepstrumCtx) return;
    const w = cepstrumCanvas.width;
    const h = cepstrumCanvas.height;
    if (w === 0 || h === 0) return;

    cepstrumCtx.fillStyle = "rgba(3, 9, 20, 0.45)";
    cepstrumCtx.fillRect(0, 0, w, h);

    if (selectedTargetId === null) {
        cepstrumCtx.fillStyle = "rgba(178, 235, 242, 0.4)";
        cepstrumCtx.font = "12px Share Tech Mono";
        cepstrumCtx.textAlign = "center";
        cepstrumCtx.textBaseline = "middle";
        cepstrumCtx.fillText("SELECT A TARGET TO VIEW CEPSTRUM & PHASE", w / 2, h / 2);
        return;
    }

    const t = targets.find(target => target.id === selectedTargetId);
    if (!t) {
        cepstrumCtx.fillStyle = "rgba(178, 235, 242, 0.4)";
        cepstrumCtx.font = "12px Share Tech Mono";
        cepstrumCtx.textAlign = "center";
        cepstrumCtx.textBaseline = "middle";
        cepstrumCtx.fillText("TARGET NOT FOUND", w / 2, h / 2);
        return;
    }

    const cep = t.cepstrum || [];
    const ph = t.unwrapped_phase || [];

    if (cep.length === 0 && ph.length === 0) {
        cepstrumCtx.fillStyle = "rgba(178, 235, 242, 0.4)";
        cepstrumCtx.font = "12px Share Tech Mono";
        cepstrumCtx.textAlign = "center";
        cepstrumCtx.textBaseline = "middle";
        cepstrumCtx.fillText("NO CEPSTRUM/PHASE DATA", w / 2, h / 2);
        return;
    }

    const padding = 20;
    const drawWidth = w - 2 * padding;
    const drawHeight = h - 2 * padding;

    // Draw grid lines
    cepstrumCtx.strokeStyle = "rgba(0, 229, 255, 0.05)";
    cepstrumCtx.lineWidth = 1;
    for (let i = 1; i < 4; i++) {
        // Horizontal grid
        const y = padding + (i / 4) * drawHeight;
        cepstrumCtx.beginPath();
        cepstrumCtx.moveTo(padding, y);
        cepstrumCtx.lineTo(w - padding, y);
        cepstrumCtx.stroke();

        // Vertical grid
        const x = padding + (i / 4) * drawWidth;
        cepstrumCtx.beginPath();
        cepstrumCtx.moveTo(x, padding);
        cepstrumCtx.lineTo(x, h - padding);
        cepstrumCtx.stroke();
    }

    function drawDataset(data, color, label, isPhase) {
        if (data.length < 2) return;
        
        let minVal = data[0];
        let maxVal = data[0];
        for (let i = 1; i < data.length; i++) {
            if (data[i] < minVal) minVal = data[i];
            if (data[i] > maxVal) maxVal = data[i];
        }

        const valRange = (maxVal - minVal) || 1.0;

        cepstrumCtx.strokeStyle = color;
        cepstrumCtx.lineWidth = 1.5;
        cepstrumCtx.beginPath();

        for (let i = 0; i < data.length; i++) {
            const x = padding + (i / (data.length - 1)) * drawWidth;
            const normVal = (data[i] - minVal) / valRange;
            const y = h - padding - normVal * drawHeight;
            if (i === 0) {
                cepstrumCtx.moveTo(x, y);
            } else {
                cepstrumCtx.lineTo(x, y);
            }
        }
        cepstrumCtx.stroke();

        cepstrumCtx.fillStyle = color;
        cepstrumCtx.font = "9px Share Tech Mono";
        cepstrumCtx.textAlign = isPhase ? "left" : "right";
        const labelY = padding + (isPhase ? 10 : 22);
        const labelX = isPhase ? padding + 10 : w - padding - 10;
        cepstrumCtx.fillText(`${label}: [${minVal.toFixed(1)}, ${maxVal.toFixed(1)}]`, labelX, labelY);
    }

    drawDataset(ph, "rgba(0, 230, 118, 0.85)", "PHASE (unwrap)", true);
    drawDataset(cep, "rgba(0, 229, 255, 0.85)", "CEPSTRUM", false);
}

drawCepstrumScope();

// Phase 3 & 4 Event Handlers
if (cicModeSelect) {
    cicModeSelect.addEventListener("change", (e) => {
        const selectValue = e.target.value;
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({
                command: "set_cic_mode",
                target_id: selectedTargetId,
                mode: selectValue
            }));
        }
    });
}

function sendStareModeCommand() {
    if (selectedTargetId === null) return;
    const checkboxState = stareModeToggle ? stareModeToggle.checked : false;
    const x = parseFloat(stareXEl ? stareXEl.value : 0) || 0.0;
    const y = parseFloat(stareYEl ? stareYEl.value : 0) || 0.0;
    const z = parseFloat(stareZEl ? stareZEl.value : 0) || 0.0;
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({
            command: "SetStareMode",
            target_id: selectedTargetId,
            coords: [x, y, z],
            enabled: checkboxState
        }));
    }
}

if (stareModeToggle) {
    stareModeToggle.addEventListener("change", sendStareModeCommand);
}
if (stareXEl) stareXEl.addEventListener("input", sendStareModeCommand);
if (stareYEl) stareYEl.addEventListener("input", sendStareModeCommand);
if (stareZEl) stareZEl.addEventListener("input", sendStareModeCommand);

if (ghostMicToggle) {
    ghostMicToggle.addEventListener("change", (e) => {
        const checkboxState = e.target.checked;
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({
                command: "SetOmniMode",
                mode: "GhostMic",
                enabled: checkboxState
            }));
            ws.send(JSON.stringify({
                command: "StartAudioStreaming"
            }));
            if (selectedTargetId !== null) {
                ws.send(JSON.stringify({
                    command: "toggle_ghost_mic",
                    target_id: selectedTargetId,
                    enabled: checkboxState
                }));
            }
        }
    });
}

if (ghostMicGain) {
    ghostMicGain.addEventListener("input", (e) => {
        const sliderValue = parseFloat(e.target.value);
        if (ghostMicGainVal) {
            ghostMicGainVal.innerText = sliderValue.toFixed(1);
        }
        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({
                command: "SetGhostMicGain",
                gain: sliderValue
            }));
        }
    });
}

// Tab navigation logic
document.querySelectorAll(".tab-btn").forEach(btn => {
    btn.addEventListener("click", () => {
        document.querySelectorAll(".tab-btn").forEach(b => b.classList.remove("active"));
        document.querySelectorAll(".tab-content").forEach(c => c.classList.remove("active"));
        
        btn.classList.add("active");
        const tabId = btn.getAttribute("data-tab");
        const content = document.getElementById(tabId);
        if (content) {
            content.classList.add("active");
        }
        
        // Trigger canvas resize to fit new tab containers
        if (typeof resizeCanvases === "function") {
            resizeCanvases();
        }
    });
});
