import numpy as np
import matplotlib.pyplot as plt

def opd_torque(throttle, speed):
    """
    Calculate the OPD torque (Nm) given a throttle position (0-100%) and vehicle speed (kph)
    based on the full OPD logic (including near-zero PID, regen, and acceleration zones).
    
    Assumptions:
      - Drive gear only (as in your case).
      - For speeds below ZERO_SPEED_THRESHOLD, a simple PID controller is simulated.
    
    Returns:
      Torque (Nm): negative torque indicates acceleration drive,
                   positive torque indicates regenerative braking.
    """
    # OPD parameters (from your C++ code)
    ZERO_SPEED_THRESHOLD = 1.0       # kph; below which PID is used
    MAX_SPEED = 120.0                # kph; maximum OPD speed
    COAST_RANGE = 10.0               # Percent: defines the width of the coast (zero-torque) zone
    PHI = 35.0                     # Percent: top speed pedal position
    SHAPE_FACTOR = 2.0             # Pedal map curve shape factor
    REGEN_TORQUE_CAP = 200.0       # Nm; cap for the PID output at near-zero speeds
    SLOW_SPEED_REGEN_CAP = 200.0   # Nm; regen torque cap for very low speeds (<10 kph)
    MAX_TRQ = 850.0              # Nm; maximum motor torque
    MAX_REGEN_PERCENT = 80.0     # Percent of MAX_TRQ used for regenerative braking calculation

    absSpeed = abs(speed)

    # --- Near-Zero Speed: Use PID Anti-Rollback Protection ---
    if absSpeed < ZERO_SPEED_THRESHOLD:
        # For simulation purposes we use a simple PID approximation.
        # Assume setpoint=0, error = 0 - speed, with a proportional gain (kp=100)
        pidOutput = -100.0 * speed  
        # Cap the PID output for safety.
        if pidOutput > REGEN_TORQUE_CAP:
            pidOutput = REGEN_TORQUE_CAP
        return pidOutput

    # --- Normal OPD Operation (speed >= ZERO_SPEED_THRESHOLD) ---
    # Compute speed as a percentage of MAX_SPEED (clipped to [0,1])
    speedPercent = np.clip(absSpeed / MAX_SPEED, 0.0, 1.0)
    
    # Determine the coast zone boundaries.
    coastUpper = PHI * (speedPercent ** (1.0 / SHAPE_FACTOR))
    coastLower = coastUpper - COAST_RANGE * speedPercent

    # --- Coast Zone: No torque is applied if throttle is within the coast boundaries ---
    if throttle >= coastLower and throttle <= coastUpper:
        return 0.0

    # --- Acceleration Zone: Throttle above the coastUpper boundary ---
    if throttle > coastUpper:
        normalizedPosition = (throttle - coastUpper) / (100.0 - coastUpper) if (100.0 - coastUpper) != 0 else 0.0
        maxTorque = MAX_TRQ
        # Scale down maximum torque when speed is low (<20 kph) to avoid overshoot.
        if absSpeed < 20.0:
            maxTorque *= (0.8 + (20.0 - absSpeed) / 100.0)
        speedFactor = 1.0 - (speedPercent ** 2)
        torque = maxTorque * speedFactor * (normalizedPosition ** 1.5)
        # In drive mode, acceleration torque is negative.
        return -torque

    # --- Regenerative Zone: Throttle below the coastLower boundary ---
    else:
        normalizedPosition = throttle / coastLower if coastLower != 0 else 0.0
        # Calculate maximum regen torque as a percentage of MAX_TRQ.
        maxRegen = MAX_REGEN_PERCENT * (MAX_TRQ / 100.0)
        # Cap regen torque at very low speeds.
        if absSpeed < 10.0:
            maxRegen = SLOW_SPEED_REGEN_CAP
        # Optionally reduce regen torque at higher speeds (>60 kph).
        if absSpeed > 60.0:
            maxRegen *= (1.0 - (absSpeed - 60.0) * 0.005)
        regenTorque = maxRegen * (1.0 - normalizedPosition)
        # Regenerative braking produces a positive torque.
        return regenTorque

# ----------------------------
# Simulation: Plotting Torque vs. Speed for various throttle positions
# ----------------------------

# Speed values from 0 to MAX_SPEED (kph)
speeds = np.linspace(0, 120, 500)

# Define a set of throttle positions (in %) for simulation
throttle_values = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100]

plt.figure(figsize=(10, 6))

# Compute and plot torque for each throttle setting over the speed range.
for throttle in throttle_values:
    torques = [opd_torque(throttle, s) for s in speeds]
    plt.plot(speeds, torques, label=f'{throttle}% Throttle')

plt.xlabel('Vehicle Speed (kph)')
plt.ylabel('Torque (Nm)')
plt.title('OPD Mode Torque vs. Speed for Various Throttle Positions')
plt.legend(title='Throttle')
plt.grid(True)
plt.show()