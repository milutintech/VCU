// ===== GLOBALS =====
let ws = null;
let wsReconnectInterval = null;
let isConnected = false;

// ===== INITIALIZATION =====
document.addEventListener('DOMContentLoaded', function() {
    initializeTabs();
    initializeSliders();
    loadConfiguration();
    connectWebSocket();
    fetchSystemInfo();

    // Refresh system info every 5 seconds
    setInterval(fetchSystemInfo, 5000);

    // Refresh CAN messages every 500ms if on CAN monitor tab
    setInterval(() => {
        const activeTab = document.querySelector('.tab-content.active');
        if (activeTab && activeTab.id === 'can-monitor') {
            fetchCANMessages();
        }
    }, 500);
});

// ===== TAB MANAGEMENT =====
function initializeTabs() {
    const tabBtns = document.querySelectorAll('.tab-btn');

    tabBtns.forEach(btn => {
        btn.addEventListener('click', () => {
            const tabName = btn.getAttribute('data-tab');
            switchTab(tabName);
        });
    });
}

function switchTab(tabName) {
    // Update tab buttons
    document.querySelectorAll('.tab-btn').forEach(btn => {
        btn.classList.remove('active');
    });
    document.querySelector(`[data-tab="${tabName}"]`).classList.add('active');

    // Update tab content
    document.querySelectorAll('.tab-content').forEach(content => {
        content.classList.remove('active');
    });
    document.getElementById(tabName).classList.add('active');

    // Load tab-specific data when switching
    if (tabName === 'can-monitor') {
        fetchCANMessages();
    } else if (tabName === 'system') {
        fetchSystemInfo();
    } else if (tabName === 'charging' || tabName === 'configuration') {
        // Reload configuration to show current saved values
        loadConfiguration();
    }
}

// ===== SLIDER INITIALIZATION =====
function initializeSliders() {
    const sliders = document.querySelectorAll('.slider');

    sliders.forEach(slider => {
        const valueDisplay = document.getElementById(slider.id + 'Value');

        if (valueDisplay) {
            slider.addEventListener('input', () => {
                valueDisplay.textContent = slider.value;
            });
        }
    });
}

// ===== WEBSOCKET CONNECTION =====
function connectWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws`;

    ws = new WebSocket(wsUrl);

    ws.onopen = () => {
        console.log('WebSocket connected');
        isConnected = true;
        updateConnectionStatus(true);

        if (wsReconnectInterval) {
            clearInterval(wsReconnectInterval);
            wsReconnectInterval = null;
        }
    };

    ws.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            updateDashboard(data);
        } catch (error) {
            console.error('Error parsing WebSocket message:', error);
        }
    };

    ws.onerror = (error) => {
        console.error('WebSocket error:', error);
    };

    ws.onclose = () => {
        console.log('WebSocket disconnected');
        isConnected = false;
        updateConnectionStatus(false);

        // Attempt reconnection every 3 seconds
        if (!wsReconnectInterval) {
            wsReconnectInterval = setInterval(() => {
                console.log('Attempting to reconnect WebSocket...');
                connectWebSocket();
            }, 3000);
        }
    };
}

function updateConnectionStatus(connected) {
    const statusEl = document.getElementById('wsStatus');
    if (connected) {
        statusEl.textContent = '● Connected';
        statusEl.classList.add('connected');
    } else {
        statusEl.textContent = '● Disconnected';
        statusEl.classList.remove('connected');
    }
}

// ===== CONFIGURATION MANAGEMENT =====
async function loadConfiguration() {
    try {
        // Load limits config (for max torque)
        try {
            const limitsRes = await fetch('/api/config/limits');
            if (!limitsRes.ok) {
                console.error('Failed to fetch limits config:', limitsRes.status, limitsRes.statusText);
            } else {
                const limitsData = await limitsRes.json();
                console.log('Loading limits config:', limitsData);

                if (document.getElementById('maxTorque')) {
                    const torqueValue = limitsData.maxTorque !== undefined ? limitsData.maxTorque : 400;
                    document.getElementById('maxTorque').value = torqueValue;
                    document.getElementById('maxTorqueValue').textContent = torqueValue;
                    console.log('Setting maxTorque slider to:', torqueValue, '(from API:', limitsData.maxTorque, ')');
                }
            }
        } catch (error) {
            console.error('Error loading limits config:', error);
        }

        // Load charging config
        try {
            const chargingRes = await fetch('/api/config/charging');
            if (!chargingRes.ok) {
                console.error('Failed to fetch charging config:', chargingRes.status, chargingRes.statusText);
            } else {
                const chargingData = await chargingRes.json();

                console.log('Loading charging config:', chargingData);

                const maxCurrentSlider = document.getElementById('maxChargingCurrent');
                const maxCurrentValue = document.getElementById('maxChargingCurrentValue');
                const targetSOCSlider = document.getElementById('chargeTargetSOC');
                const targetSOCValue = document.getElementById('chargeTargetSOCValue');

                if (maxCurrentSlider && maxCurrentValue) {
                    const currentValue = chargingData.maxChargingCurrent !== undefined ? chargingData.maxChargingCurrent : 32;
                    maxCurrentSlider.value = currentValue;
                    maxCurrentValue.textContent = currentValue;
                    console.log('Setting maxChargingCurrent slider to:', currentValue, '(from API:', chargingData.maxChargingCurrent, ')');
                }

                if (targetSOCSlider && targetSOCValue) {
                    const socValue = chargingData.maxSOC !== undefined ? chargingData.maxSOC : 100;
                    targetSOCSlider.value = socValue;
                    targetSOCValue.textContent = socValue;
                    console.log('Setting maxSOC slider to:', socValue, '(from API:', chargingData.maxSOC, ')');
                }
            }
        } catch (error) {
            console.error('Error loading charging config:', error);
        }

        // Load pedal config
        try {
            const pedalRes = await fetch('/api/config/pedal');
            const pedalData = await pedalRes.json();

            document.getElementById('regenZoneEnd').value = pedalData.regenZoneEnd || 16;
            document.getElementById('regenZoneEndValue').textContent = (pedalData.regenZoneEnd || 16).toFixed(1);
            document.getElementById('coastZoneEnd').value = pedalData.coastZoneEnd || 17;
            document.getElementById('coastZoneEndValue').textContent = (pedalData.coastZoneEnd || 17).toFixed(1);
            document.getElementById('regenProgression').value = pedalData.regenProgression || 1.5;
            document.getElementById('regenProgressionValue').textContent = (pedalData.regenProgression || 1.5).toFixed(1);
            document.getElementById('accelProgression').value = pedalData.accelProgression || 1.7;
            document.getElementById('accelProgressionValue').textContent = (pedalData.accelProgression || 1.7).toFixed(1);
        } catch (error) {
            console.error('Error loading pedal config:', error);
        }

        // Load transition config
        try {
            const transitionRes = await fetch('/api/config/transition');
            const transitionData = await transitionRes.json();

            document.getElementById('regenEngageTime').value = transitionData.regenEngageTime || 300;
            document.getElementById('regenEngageTimeValue').textContent = transitionData.regenEngageTime || 300;
            document.getElementById('regenReleaseTime').value = transitionData.regenReleaseTime || 150;
            document.getElementById('regenReleaseTimeValue').textContent = transitionData.regenReleaseTime || 150;
            document.getElementById('powerEngageTime').value = transitionData.powerEngageTime || 200;
            document.getElementById('powerEngageTimeValue').textContent = transitionData.powerEngageTime || 200;
            document.getElementById('powerReleaseTime').value = transitionData.powerReleaseTime || 100;
            document.getElementById('powerReleaseTimeValue').textContent = transitionData.powerReleaseTime || 100;
            document.getElementById('crossoverTime').value = transitionData.crossoverTime || 400;
            document.getElementById('crossoverTimeValue').textContent = transitionData.crossoverTime || 400;
        } catch (error) {
            console.error('Error loading transition config:', error);
        }

    } catch (error) {
        console.error('Error loading configuration:', error);
    }
}

// ===== SAVE CONFIGURATION FUNCTIONS =====
async function saveMotorConfig() {
    const config = {
        maxTorque: parseInt(document.getElementById('maxTorque').value)
    };

    try {
        const res = await fetch('/api/config/limits', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(config)
        });

        const result = await res.json();
        if (result.success) {
            alert('Motor configuration saved successfully!');
            // Reload configuration to show saved values
            loadConfiguration();
        } else {
            alert('Error saving configuration: ' + (result.error || 'Unknown error'));
        }
    } catch (error) {
        console.error('Error saving motor config:', error);
        alert('Failed to save configuration');
    }
}

async function saveChargingConfig() {
    const config = {
        maxChargingCurrent: parseInt(document.getElementById('maxChargingCurrent').value),
        maxSOC: parseInt(document.getElementById('chargeTargetSOC').value)
    };

    try {
        const res = await fetch('/api/config/charging', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(config)
        });

        const result = await res.json();
        if (result.success) {
            alert('Charging configuration saved successfully!');
            // Reload configuration to show saved values
            loadConfiguration();
        } else {
            alert('Error saving configuration: ' + (result.error || 'Unknown error'));
        }
    } catch (error) {
        console.error('Error saving charging config:', error);
        alert('Failed to save configuration');
    }
}

async function savePedalConfig() {
    const config = {
        regenZoneEnd: parseFloat(document.getElementById('regenZoneEnd').value),
        coastZoneEnd: parseFloat(document.getElementById('coastZoneEnd').value),
        regenProgression: parseFloat(document.getElementById('regenProgression').value),
        accelProgression: parseFloat(document.getElementById('accelProgression').value)
    };

    try {
        const res = await fetch('/api/config/pedal', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(config)
        });

        const result = await res.json();
        if (result.success) {
            alert('Pedal configuration saved successfully!');
            // Reload configuration to show saved values
            loadConfiguration();
        } else {
            alert('Error saving configuration: ' + (result.error || 'Unknown error'));
        }
    } catch (error) {
        console.error('Error saving pedal config:', error);
        alert('Failed to save configuration');
    }
}

async function saveTransitionConfig() {
    const config = {
        regenEngageTime: parseFloat(document.getElementById('regenEngageTime').value),
        regenReleaseTime: parseFloat(document.getElementById('regenReleaseTime').value),
        powerEngageTime: parseFloat(document.getElementById('powerEngageTime').value),
        powerReleaseTime: parseFloat(document.getElementById('powerReleaseTime').value),
        crossoverTime: parseFloat(document.getElementById('crossoverTime').value)
    };

    try {
        const res = await fetch('/api/config/transition', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(config)
        });

        const result = await res.json();
        if (result.success) {
            alert('Transition configuration saved successfully!');
            // Reload configuration to show saved values
            loadConfiguration();
        } else {
            alert('Error saving configuration: ' + (result.error || 'Unknown error'));
        }
    } catch (error) {
        console.error('Error saving transition config:', error);
        alert('Failed to save configuration');
    }
}

// ===== SYSTEM FUNCTIONS =====
async function saveDebugMode() {
    const enabled = document.getElementById('debugMode').checked;

    try {
        const res = await fetch('/api/system/debug', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ enabled })
        });

        const result = await res.json();
        if (result.success) {
            alert('Debug mode ' + (enabled ? 'enabled' : 'disabled'));
        }
    } catch (error) {
        console.error('Error setting debug mode:', error);
        alert('Failed to set debug mode');
    }
}

async function exportConfig() {
    try {
        const res = await fetch('/api/system/export');
        const config = await res.json();

        const dataStr = JSON.stringify(config, null, 2);
        const dataBlob = new Blob([dataStr], { type: 'application/json' });
        const url = URL.createObjectURL(dataBlob);

        const link = document.createElement('a');
        link.href = url;
        link.download = 'vcu_config.json';
        link.click();

        URL.revokeObjectURL(url);
    } catch (error) {
        console.error('Error exporting config:', error);
        alert('Failed to export configuration');
    }
}

async function importConfig() {
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = 'application/json';

    input.onchange = async (e) => {
        const file = e.target.files[0];
        const text = await file.text();

        try {
            const config = JSON.parse(text);

            const res = await fetch('/api/system/import', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(config)
            });

            const result = await res.json();
            if (result.success) {
                alert('Configuration imported successfully!');
                loadConfiguration();
            } else {
                alert('Error importing configuration: ' + (result.error || 'Unknown error'));
            }
        } catch (error) {
            console.error('Error importing config:', error);
            alert('Failed to import configuration');
        }
    };

    input.click();
}

async function resetConfig() {
    if (!confirm('Are you sure you want to reset to factory defaults? This cannot be undone.')) {
        return;
    }

    try {
        const res = await fetch('/api/system/reset', { method: 'POST' });
        const result = await res.json();

        if (result.success) {
            alert('Configuration reset to factory defaults');
            loadConfiguration();
        }
    } catch (error) {
        console.error('Error resetting config:', error);
        alert('Failed to reset configuration');
    }
}

async function fetchSystemInfo() {
    try {
        const res = await fetch('/api/system/info');
        const data = await res.json();

        document.getElementById('chipModel').textContent = data.chipModel || '-';
        document.getElementById('sysInfoFreeHeap').textContent = formatBytes(data.freeHeap || 0);
        document.getElementById('sysInfoUptime').textContent = formatUptime(data.uptime || 0);
        document.getElementById('debugMode').checked = data.debugMode || false;
    } catch (error) {
        console.error('Error fetching system info:', error);
    }
}

async function fetchCANMessages() {
    try {
        const res = await fetch('/api/can/messages');
        const data = await res.json();

        const container = document.getElementById('canMessages');
        container.innerHTML = '';

        if (data.messages && data.messages.length > 0) {
            data.messages.forEach(msg => {
                const div = document.createElement('div');
                div.className = 'can-message';
                div.innerHTML = `<strong>${msg.id}</strong> - ${msg.name}: ${JSON.stringify(msg, null, 2)}`;
                container.appendChild(div);
            });
        } else {
            container.innerHTML = '<div class="can-message">No CAN messages available</div>';
        }
    } catch (error) {
        console.error('Error fetching CAN messages:', error);
    }
}

// ===== UTILITY FUNCTIONS =====
function formatBytes(bytes) {
    if (bytes < 1024) return bytes + ' B';
    if (bytes < 1048576) return (bytes / 1024).toFixed(2) + ' KB';
    return (bytes / 1048576).toFixed(2) + ' MB';
}

function formatUptime(seconds) {
    const days = Math.floor(seconds / 86400);
    const hours = Math.floor((seconds % 86400) / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    const secs = seconds % 60;

    if (days > 0) {
        return `${days}d ${hours}h ${minutes}m`;
    } else if (hours > 0) {
        return `${hours}h ${minutes}m ${secs}s`;
    } else {
        return `${minutes}m ${secs}s`;
    }
}
