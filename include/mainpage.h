/**
 * @mainpage Vehicle Control Unit Documentation
 * 
 * @section intro Introduction
 * This is the documentation for the ESP32-based Vehicle Control Unit (VCU).
 * The system manages all aspects of electric vehicle control including:
 * 
 * - Motor control and torque management
 * - Battery system monitoring and protection
 * - Charging system control
 * - Vehicle state management
 * - Real-time diagnostics and monitoring
 * 
 * @section arch System Architecture
 * The VCU runs on a dual-core ESP32:
 * - Core 0: CAN communication and fast control loops
 * - Core 1: State management and system monitoring
 * 
 * @section modules Main Modules
 * - StateManager: Vehicle state and safety management
 * - CANManager: Communication with vehicle subsystems
 * - VehicleControl: Motor control and driving dynamics
 * - SerialConsole: Debug interface and monitoring
 * 
 * @section safety Safety Systems
 * The VCU implements multiple safety systems:
 * - Battery voltage and current protection
 * - Temperature monitoring and management
 * - Precharge sequence control
 * - Watchdog monitoring
 * - Error detection and handling
 * 
 * @section config Configuration
 * System parameters are defined in vehicle_parameters.h and include:
 * - Battery limits and thresholds
 * - Motor performance parameters
 * - Temperature limits
 * - Control response curves
 */