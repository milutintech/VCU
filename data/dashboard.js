// ===== CHART GLOBALS =====
let speedChart = null;
let powerChart = null;
let speedData = [];
let powerData = [];
const MAX_DATA_POINTS = 100;

// ===== INITIALIZE CHARTS =====
document.addEventListener('DOMContentLoaded', function() {
    initializeCharts();
});

function initializeCharts() {
    const speedCanvas = document.getElementById('speedChart');
    const powerCanvas = document.getElementById('powerChart');

    if (speedCanvas && powerCanvas) {
        speedChart = speedCanvas.getContext('2d');
        powerChart = powerCanvas.getContext('2d');

        // Set canvas size
        speedCanvas.width = speedCanvas.offsetWidth;
        speedCanvas.height = 250;
        powerCanvas.width = powerCanvas.offsetWidth;
        powerCanvas.height = 250;

        // Initialize data arrays
        for (let i = 0; i < MAX_DATA_POINTS; i++) {
            speedData.push(0);
            powerData.push(0);
        }

        // Start animation loop
        requestAnimationFrame(drawCharts);
    }
}

function drawCharts() {
    if (speedChart) {
        drawChart(speedChart, speedData, '#4caf50', 'Speed');
    }
    if (powerChart) {
        drawChart(powerChart, powerData, '#6b9bd4', 'Power');
    }

    requestAnimationFrame(drawCharts);
}

function drawChart(ctx, data, color, label) {
    const canvas = ctx.canvas;
    const width = canvas.width;
    const height = canvas.height;
    const padding = 20;

    // Clear canvas
    ctx.fillStyle = '#1a1a1a';
    ctx.fillRect(0, 0, width, height);

    // Find min/max for scaling
    const max = Math.max(...data, 1);
    const min = Math.min(...data, 0);
    const range = max - min || 1;

    // Draw grid
    ctx.strokeStyle = '#3a3a3a';
    ctx.lineWidth = 1;

    // Horizontal grid lines
    for (let i = 0; i <= 4; i++) {
        const y = padding + (height - 2 * padding) * (i / 4);
        ctx.beginPath();
        ctx.moveTo(padding, y);
        ctx.lineTo(width - padding, y);
        ctx.stroke();
    }

    // Draw data line
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();

    const pointSpacing = (width - 2 * padding) / (MAX_DATA_POINTS - 1);

    for (let i = 0; i < data.length; i++) {
        const x = padding + i * pointSpacing;
        const normalized = (data[i] - min) / range;
        const y = (height - padding) - (normalized * (height - 2 * padding));

        if (i === 0) {
            ctx.moveTo(x, y);
        } else {
            ctx.lineTo(x, y);
        }
    }

    ctx.stroke();

    // Draw fill gradient
    ctx.lineTo(width - padding, height - padding);
    ctx.lineTo(padding, height - padding);
    ctx.closePath();

    const gradient = ctx.createLinearGradient(0, padding, 0, height - padding);
    gradient.addColorStop(0, color + '40');  // 40 = 25% opacity in hex
    gradient.addColorStop(1, color + '00');  // 00 = 0% opacity

    ctx.fillStyle = gradient;
    ctx.fill();

    // Draw max value label (top left)
    ctx.fillStyle = color;
    ctx.font = '12px monospace';
    ctx.fillText(max.toFixed(1), padding + 5, padding + 15);

    // Draw min value label (bottom left)
    if (min < 0) {
        ctx.fillText(min.toFixed(1), padding + 5, height - padding - 5);
    }
}

// ===== UPDATE DASHBOARD FROM WEBSOCKET =====
function updateDashboard(data) {
    // Update header
    if (data.state) {
        document.getElementById('headerState').textContent = data.state;
    }

    // Update vehicle status
    if (data.gear) {
        document.getElementById('currentGear').textContent = data.gear;
    }

    if (data.vehicle) {
        const speed = data.vehicle.speed || 0;
        const power = data.vehicle.power || 0;
        const torque = data.vehicle.torqueDemand || 0;

        document.getElementById('vehicleSpeed').textContent = speed.toFixed(0);
        document.getElementById('vehiclePower').textContent = power.toFixed(1);
        document.getElementById('torqueDemand').textContent = torque.toFixed(0);

        // Update chart data
        updateChartData(speedData, speed);
        updateChartData(powerData, power);

        // Update chart value displays
        document.getElementById('speedChartValue').textContent = speed.toFixed(1);
        document.getElementById('powerChartValue').textContent = power.toFixed(1);

        // Color code power
        const powerEl = document.getElementById('vehiclePower');
        powerEl.className = 'data-value';
        if (power > 0) {
            powerEl.classList.add('value-positive');
        } else if (power < 0) {
            powerEl.classList.add('value-negative');
        } else {
            powerEl.classList.add('value-neutral');
        }
    }

    // Update battery
    if (data.battery) {
        document.getElementById('batterySoc').textContent = (data.battery.soc || 0).toFixed(0);
        document.getElementById('batteryVoltage').textContent = (data.battery.voltage || 0).toFixed(0);

        const current = data.battery.current || 0;
        const currentEl = document.getElementById('batteryCurrent');
        currentEl.innerHTML = current.toFixed(1) + ' <span class="unit">A</span>';

        // Color code current
        currentEl.className = 'data-value';
        if (current > 0) {
            currentEl.classList.add('value-negative');  // Discharging
        } else if (current < 0) {
            currentEl.classList.add('value-positive');  // Charging
        } else {
            currentEl.classList.add('value-neutral');
        }

        document.getElementById('minCell').textContent = (data.battery.minCell || 0).toFixed(2);
    }

    // Update temperatures
    if (data.temperatures) {
        document.getElementById('tempMotor').textContent = (data.temperatures.motor || 0).toFixed(0);
        document.getElementById('tempInverter').textContent = (data.temperatures.inverter || 0).toFixed(0);
        document.getElementById('tempBattery').textContent = (data.temperatures.battery || 0).toFixed(0);
    }

    // Update inputs
    if (data.inputs) {
        document.getElementById('inputThrottle').textContent = (data.inputs.throttle || 0).toFixed(0);
        document.getElementById('inputTorque').textContent = (data.inputs.torque || 0).toFixed(0);

        document.getElementById('inputConnectorLock').textContent = data.inputs.connectorLock ? 'Locked' : 'Unlocked';
        document.getElementById('inputConnectorLock').className = 'data-value status-badge ' +
            (data.inputs.connectorLock ? 'value-positive' : 'value-neutral');

        document.getElementById('inputIgnition').textContent = data.inputs.ignition ? 'On' : 'Off';
        document.getElementById('inputIgnition').className = 'data-value status-badge ' +
            (data.inputs.ignition ? 'value-positive' : 'value-neutral');
    }

    // Update safety warnings
    if (data.safety && data.safety.throttleBlockingShift) {
        document.getElementById('throttleShiftError').style.display = 'flex';
    } else {
        document.getElementById('throttleShiftError').style.display = 'none';
    }

    // Update charging status (if elements exist on charging tab)
    if (data.charging) {
        const chargerStateEl = document.getElementById('chargerState');
        const chargerCurrentEl = document.getElementById('chargerCurrent');

        if (chargerStateEl) {
            chargerStateEl.textContent = data.charging.state || 'UNKNOWN';
        }
        if (chargerCurrentEl) {
            chargerCurrentEl.textContent = (data.charging.current || 0).toFixed(1) + ' A';
        }
    }

    // Update uptime in header
    if (data.uptime !== undefined) {
        // This will be set from the status endpoint
        const uptime = data.uptime || 0;
        const hours = Math.floor(uptime / 3600);
        const minutes = Math.floor((uptime % 3600) / 60);
        const seconds = uptime % 60;
        document.getElementById('headerUptime').textContent = `${hours}h ${minutes}m ${seconds}s`;
    }
}

function updateChartData(dataArray, newValue) {
    // Shift old data and add new value
    dataArray.shift();
    dataArray.push(newValue);
}

// ===== FETCH LIVE DATA (FALLBACK IF WEBSOCKET FAILS) =====
async function fetchLiveData() {
    try {
        const res = await fetch('/api/status/live');
        const data = await res.json();
        updateDashboard(data);
    } catch (error) {
        console.error('Error fetching live data:', error);
    }
}

// Fallback polling if WebSocket isn't working
setInterval(() => {
    if (!isConnected) {
        fetchLiveData();
    }
}, 1000);
