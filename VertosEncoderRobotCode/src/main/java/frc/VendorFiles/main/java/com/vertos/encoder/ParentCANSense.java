// Example usage of the event-driven ParentCANSense

public class ArmSubsystem {
    private ParentCANSense armEncoder;
    private double targetPosition = 0.0;
    private boolean hasTarget = false;
    private double positionTolerance = 2.0; // degrees
    private double maxSafeVelocity = 180.0; // degrees per second
    private boolean emergencyStop = false;
    
    public ArmSubsystem(int encoderDeviceID) {
        // Create the encoder with debug enabled
        armEncoder = new ParentCANSense(encoderDeviceID, true);
        
        // Set up all event listeners
        setupEventListeners();
        
        // Start the event-driven system
        armEncoder.start();
        
        System.out.println("Arm subsystem initialized with event-driven encoder");
    }
    
    private void setupEventListeners() {
        // Position updates - fires every time new position data arrives
        armEncoder.onPositionUpdate((absoluteRotations) -> {
            double degrees = absoluteRotations * 360.0;
            
            // Check if we've reached our target
            if (hasTarget && Math.abs(degrees - targetPosition) < positionTolerance) {
                System.out.printf("Target reached! Current: %.2f°, Target: %.2f°%n", 
                                degrees, targetPosition);
                onTargetReached();
            }
            
            // Safety check - position limits
            if (degrees < -90 || degrees > 270) {
                System.out.printf("WARNING: Arm position %.2f° is outside safe range!%n", degrees);
                triggerSafetyStop("Position limit exceeded");
            }
            
            // Update dashboard or other systems
            updateDashboard("arm_position", degrees);
        });
        
        // Velocity updates - fires every time new velocity data arrives
        armEncoder.onVelocityUpdate((velocityRPS) -> {
            double degreesPerSecond = velocityRPS * 360.0;
            
            // Safety check for over-speed
            if (Math.abs(degreesPerSecond) > maxSafeVelocity) {
                System.out.printf("EMERGENCY: Over-speed detected! %.1f°/s (limit: %.1f°/s)%n", 
                                Math.abs(degreesPerSecond), maxSafeVelocity);
                triggerSafetyStop("Over-speed detected");
            }
            
            // Check if arm has stopped moving (for movement completion detection)
            if (hasTarget && Math.abs(degreesPerSecond) < 5.0) { // Less than 5°/s considered stopped
                checkIfTargetReached();
            }
            
            updateDashboard("arm_velocity", degreesPerSecond);
        });
        
        // Acceleration updates
        armEncoder.onAccelerationUpdate((accelerationRPS2) -> {
            double degreesPerSecond2 = accelerationRPS2 * 360.0;
            
            // Log high acceleration events
            if (Math.abs(degreesPerSecond2) > 720.0) { // 2 full rotations/s²
                System.out.printf("High acceleration detected: %.1f°/s²%n", degreesPerSecond2);
            }
            
            updateDashboard("arm_acceleration", degreesPerSecond2);
        });
        
        // Fault status updates
        armEncoder.onFaultStatusUpdate((faultByte) -> {
            System.out.printf("Fault status byte updated: 0x%02X%n", faultByte);
            
            // Check for critical faults
            if (armEncoder.getStickyFault_Hardware()) {
                System.out.println("CRITICAL: Hardware fault detected on arm encoder!");
                triggerSafetyStop("Hardware fault");
            }
            
            if (armEncoder.getStickyFault_BadMagnet()) {
                System.out.println("WARNING: Bad magnet fault - encoder may be unreliable");
            }
        });
        
        // Connection state monitoring
        armEncoder.onConnectionChanged((connected) -> {
            if (connected) {
                System.out.println("Arm encoder reconnected - resuming normal operation");
                // Could re-home or verify position here
                emergencyStop = false;
            } else {
                System.out.println("Arm encoder disconnected - switching to safe mode");
                triggerSafetyStop("Encoder disconnected");
            }
        });
        
        // Core device fault monitoring
        armEncoder.onFaultChanged((faulted) -> {
            if (faulted) {
                System.out.println("Core device fault on arm encoder - limiting operations");
                triggerSafetyStop("Core device fault");
            } else {
                System.out.println("Core device fault cleared on arm encoder");
                // Could resume operations here if other conditions are met
            }
        });
    }
    
    // Command methods that can be called from robot code
    public void moveToPosition(double targetDegrees) {
        if (emergencyStop) {
            System.out.println("Cannot move - emergency stop active");
            return;
        }
        
        targetPosition = targetDegrees;
        hasTarget = true;
        
        System.out.printf("Moving arm to %.2f degrees (current: %.2f)%n", 
                         targetDegrees, armEncoder.getAbsPositionDegrees());
        
        // Here you would send commands to your motor controller
        // The encoder events will automatically track progress
    }
    
    public void stop() {
        hasTarget = false;
        System.out.println("Arm movement stopped");
        // Send stop command to motor controller
    }
    
    public void zeroEncoder() {
        System.out.println("Zeroing arm encoder...");
        if (armEncoder.zeroEncoder()) {
            System.out.println("Encoder zeroed successfully");
        } else {
            System.out.println("Failed to zero encoder");
        }
    }
    
    public void clearFaults() {
        System.out.println("Clearing encoder faults...");
        armEncoder.resetStickyFaults();
        emergencyStop = false;
    }
    
    private void onTargetReached() {
        hasTarget = false;
        System.out.println("Arm has reached target position!");
        
        // Could trigger completion callbacks here
        // notifyMovementComplete();
    }
    
    private void checkIfTargetReached() {
        if (hasTarget) {
            double currentPosition = armEncoder.getAbsPositionDegrees();
            if (Math.abs(currentPosition - targetPosition) < positionTolerance) {
                onTargetReached();
            }
        }
    }
    
    private void triggerSafetyStop(String reason) {
        emergencyStop = true;
        hasTarget = false;
        
        System.out.printf("SAFETY STOP TRIGGERED: %s%n", reason);
        
        // Send emergency stop to motor controller
        // activateEmergencyStop();
        
        // Could also trigger robot-wide emergency stop
        // Robot.emergencyStop();
    }
    
    // Getters for current state (all non-blocking and thread-safe)
    public double getCurrentPosition() {
        return armEncoder.getAbsPositionDegrees();
    }
    
    public double getCurrentVelocity() {
        return armEncoder.getVelocityDegreesPerSecond();
    }
    
    public boolean isMoving() {
        return Math.abs(armEncoder.getVelocityDegreesPerSecond()) > 1.0; // > 1°/s
    }
    
    public boolean hasTarget() {
        return hasTarget;
    }
    
    public boolean isEmergencyStop() {
        return emergencyStop;
    }
    
    public boolean isConnected() {
        return armEncoder.isConnected();
    }
    
    public boolean hasFaults() {
        return armEncoder.hasAnyFault();
    }
    
    public String getStatus() {
        return armEncoder.getDiagnosticInfo();
    }
    
    // Mock dashboard update method
    private void updateDashboard(String key, double value) {
        // In real code, this would update NetworkTables or similar
        // SmartDashboard.putNumber(key, value);
        System.out.printf("Dashboard: %s = %.2f%n", key, value);
    }
    
    // Clean shutdown
    public void shutdown() {
        armEncoder.stop();
        System.out.println("Arm subsystem shutdown complete");
    }
}

// Example robot class showing integration
public class EventDrivenRobot extends TimedRobot {
    private ArmSubsystem armSubsystem;
    private Joystick operatorJoystick;
    
    @Override
    public void robotInit() {
        // Initialize subsystems
        armSubsystem = new ArmSubsystem(5); // Device ID 5
        operatorJoystick = new Joystick(1);
        
        System.out.println("Event-driven robot initialized");
    }
    
    @Override
    public void teleopPeriodic() {
        // All encoder data is automatically updated via events!
        // No need to manually poll anything
        
        // Example joystick controls
        if (operatorJoystick.getRawButtonPressed(1)) {
            armSubsystem.moveToPosition(90.0); // Move to 90 degrees
        }
        
        if (operatorJoystick.getRawButtonPressed(2)) {
            armSubsystem.moveToPosition(0.0); // Move to home position
        }
        
        if (operatorJoystick.getRawButtonPressed(3)) {
            armSubsystem.stop(); // Stop movement
        }
        
        if (operatorJoystick.getRawButtonPressed(4)) {
            armSubsystem.zeroEncoder(); // Zero the encoder
        }
        
        if (operatorJoystick.getRawButtonPressed(5)) {
            armSubsystem.clearFaults(); // Clear any faults
        }
        
        // Display current status periodically
        if (Timer.getFPGATimestamp() % 2.0 < 0.02) { // Every 2 seconds
            System.out.printf("Arm Status: Position=%.1f°, Velocity=%.1f°/s, Moving=%s, Faults=%s%n",
                             armSubsystem.getCurrentPosition(),
                             armSubsystem.getCurrentVelocity(),
                             armSubsystem.isMoving() ? "Yes" : "No",
                             armSubsystem.hasFaults() ? "Yes" : "No");
        }
    }
    
    @Override
    public void autonomousInit() {
        // Example autonomous sequence
        armSubsystem.moveToPosition(45.0);
    }
    
    @Override
    public void disabledInit() {
        // Clean shutdown when disabled
        armSubsystem.shutdown();
    }
    
    @Override
    public void robotPeriodic() {
        // Robot periodic still runs at 50Hz
        // But encoder updates can happen at 1000Hz via events!
        // This gives much better responsiveness for safety systems
    }
}