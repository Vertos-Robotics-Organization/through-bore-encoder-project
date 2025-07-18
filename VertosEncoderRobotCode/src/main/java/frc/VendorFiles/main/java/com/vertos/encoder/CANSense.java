package frc.VendorFiles.main.java.com.vertos.encoder;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class CANSense extends CoreDevice implements Sendable, AutoCloseable {

    public boolean debugMode;
    public final int deviceID;
    
    // Native encoder data (received from CAN)
    private long multiTurnCounts;        // Native encoder counts (64-bit signed)
    private int velocityCPS;             // Velocity in counts per second (32-bit signed)
    private int accelerationCPS2;        // Acceleration in counts per second squared (32-bit signed)
        
    // Converted data (calculated from native counts)
    private double absoluteRotations;    // Calculated from multiTurnCounts
    private double relativeRotations;    // Calculated from absoluteRotations
    private double velocityRPS;          // Calculated from velocityCPS
    private double accelerationRPS2;     // Calculated from accelerationCPS2
    
    // Constants for the encoder
    private final double CountsPerRevolution = 2097152.0; // 2 ^ 21
    
    private static final int POSITION_API_ID = 0;          // API ID for position data (8 bytes - 64-bit counts)
    private static final int VELOCITY_ACCEL_API_ID = 16;    // API ID for combined velocity and acceleration data (8 bytes)
    private static final int BOOLEAN_STATUS_API_ID = 32;    // API ID for boolean status messages (1 byte)

    // Command API IDs - CORRECTED to match the C code exactly
    private static final int ZERO_ENCODER_API_ID = 10;        // API ID for zero encoder command
    private static final int SET_DIRECTION_API_ID = 11;    // API ID for set direction clockwise
    private static final int SET_POSITION_API_ID = 12;        // API ID for set position command
    private static final int RESET_FACTORY_API_ID = 13;      // API ID for factory reset command (if implemented in C)

    // Command execution state tracking
    private boolean isCommandInProgress = false;
    private long lastCommandTime = 0;
    private static final long COMMAND_TIMEOUT_MS = 5000; // 5 second timeout for commands


    public final CoreDeviceFaultLayer FAULT;

    /**
     * Constructor for CANSense.
     *
     * @param deviceID   The CAN device ID for the encoder.
     * @param debugMode  If true, enables debug output to the console.
     */
    public CANSense(int deviceID, boolean debugMode) {
        super(deviceID, debugMode);
        this.deviceID = deviceID;
        this.debugMode = debugMode;
        FAULT = new CoreDeviceFaultLayer();
        addListener(positionListener, POSITION_API_ID);
        addListener(velocityAccelListener, VELOCITY_ACCEL_API_ID);
        addListener(FAULT.getFaultListener(), BOOLEAN_STATUS_API_ID);
    }

    /**
     * Checks if a command has timed out and resets the command state if necessary.
     */
    private void checkCommandTimeout() {
        if (isCommandInProgress && (System.currentTimeMillis() - lastCommandTime) > COMMAND_TIMEOUT_MS) {
            if (debugMode) {
                System.out.printf("Device %d: Command timeout detected, resetting command state%n", deviceID);
            }
            isCommandInProgress = false;
        }
    }

    /**
     * Waits for the encoder to be ready before executing a command.
     * 
     * @param timeoutMs Maximum time to wait in milliseconds
     * @return True if encoder is ready, false if timeout
     */
    private boolean waitForReady(long timeoutMs) {
        long startTime = System.currentTimeMillis();
        
        while (System.currentTimeMillis() - startTime < timeoutMs) {
            if (!isCommandInProgress) {
                return true;
            }
            
            try {
                Thread.sleep(10); // Small delay
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return false;
            }
        }
        
        return false;
    }

    //---------------------------------------------------------------------------------------
    // Getters for converted rotation data (UPDATED but kept for compatibility)
    //---------------------------------------------------------------------------------------
    /**
     * Returns the absolute rotations calculated from encoder counts.
     *
     * @return The absolute rotations as a double.
     */
    public double getAbsRotations() {
        return absoluteRotations;
    }

    /**
     * Returns the relative rotations (0.0 to 1.0) calculated from absolute rotations.
     *
     * @return The relative rotations as a double.
     */
    public double getRotations() {
        return relativeRotations;
    }

    /**
     * Returns the sensor velocity in rotations per second.
     *
     * @return The sensor velocity in RPS as a double.
     */
    public double getSensorVelocityRPS() {
        return velocityRPS;
    }

    /**
     * Returns the sensor acceleration in rotations per second squared.
     *
     * @return The sensor acceleration in RPS² as a double.
     */
    public double getSensorAccelerationRPS2() {
        return accelerationRPS2;
    }

    //----------------------------------------------------------------------------------------
    // Additional conversion methods (NEW)
    //----------------------------------------------------------------------------------------
    
    /**
     * Returns the absolute position in degrees.
     *
     * @return The absolute position in degrees.
     */
    public double getAbsPositionDegrees() {
        return absoluteRotations * 360.0;
    }
    
    /**
     * Returns the relative position in degrees (0.0 to 360.0).
     *
     * @return The relative position in degrees.
     */
    public double getPositionDegrees() {
        return relativeRotations * 360.0;
    }
    
    /**
     * Returns the absolute position in radians.
     *
     * @return The absolute position in radians.
     */
    public double getAbsPositionRadians() {
        return absoluteRotations * 2.0 * Math.PI;
    }
    
    /**
     * Returns the relative position in radians (0.0 to 2π).
     *
     * @return The relative position in radians.
     */
    public double getPositionRadians() {
        return relativeRotations * 2.0 * Math.PI;
    }
    
    /**
     * Returns the velocity in degrees per second.
     *
     * @return The velocity in degrees per second.
     */
    public double getVelocityDegreesPerSecond() {
        return velocityRPS * 360.0;
    }
    
    /**
     * Returns the velocity in radians per second.
     *
     * @return The velocity in radians per second.
     */
    public double getVelocityRadiansPerSecond() {
        return velocityRPS * 2.0 * Math.PI;
    }
    
    /**
     * Returns the velocity in RPM (rotations per minute).
     *
     * @return The velocity in RPM.
     */
    public double getVelocityRPM() {
        return velocityRPS * 60.0;
    }



    //-------------------------------------------------------------------------------------------
    // Senders - FIXED IMPLEMENTATIONS WITH PROPER CAN COMMUNICATION
    //-------------------------------------------------------------------------------------------
    /**
     * Sends a command to zero the encoder.
     * Sets the current position as the new zero reference point.
     * 
     * @return True if the command was sent successfully, false otherwise.
     */
    public boolean zeroEncoder() {
        if (!waitForReady(1000)) {
            if (debugMode) {
                System.out.printf("Device %d: Encoder not ready for zero command%n", deviceID);
            }
            return false;
        }

        isCommandInProgress = true;
        lastCommandTime = System.currentTimeMillis();

        boolean success = sendDoubleWithTimeoutCommand(ZERO_ENCODER_API_ID, 0.0, COMMAND_TIMEOUT_MS, "Zero Encoder");
        
        if (success) {
            if (debugMode) {
                System.out.printf("Device %d: Zero encoder command sent successfully%n", deviceID);
            }
            
            // Since C code doesn't send acknowledgments, we'll just wait a bit and assume success
            try {
                Thread.sleep(100); // Small delay to allow command processing
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            
            isCommandInProgress = false;
            return true;
        }

        isCommandInProgress = false;
        return success;
    }

    /**
     * Sets the direction of the encoder to counter-clockwise.
     * Changes the sign of position, velocity, and acceleration readings.
     * 
     * @return True if the command was sent successfully, false otherwise.
     */
    public boolean setDirectionCounterClockWise() {
        if (!waitForReady(1000)) {
            if (debugMode) {
                System.out.printf("Device %d: Encoder not ready for set direction counter-clockwise command%n", deviceID);
            }
            return false;
        }

        isCommandInProgress = true;
        lastCommandTime = System.currentTimeMillis();

        // counter-clockwise is true
        // clockwise is false
        boolean success = sendBooleanWithTimeoutCommand(SET_DIRECTION_API_ID, true, COMMAND_TIMEOUT_MS, "Set Direction Counter-Clockwise");
        
        if (success) {
            if (debugMode) {
                System.out.printf("Device %d: Set direction counter-clockwise command sent successfully%n", deviceID);
            }
            
            try {
                Thread.sleep(100); // Small delay to allow command processing
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            
            isCommandInProgress = false;
            return true;
        }

        isCommandInProgress = false;
        return success;
    }

    /**
     * Sets the direction of the encoder to clockwise.
     * Changes the sign of position, velocity, and acceleration readings.
     * 
     * @return True if the command was sent successfully, false otherwise.
     */
    public boolean setDirectionClockWise() {
        if (!waitForReady(1000)) {
            if (debugMode) {
                System.out.printf("Device %d: Encoder not ready for set direction clockwise command%n", deviceID);
            }
            return false;
        }

        isCommandInProgress = true;
        lastCommandTime = System.currentTimeMillis();

        boolean success = sendBooleanWithTimeoutCommand(SET_DIRECTION_API_ID, false, COMMAND_TIMEOUT_MS, "Set Direction Clockwise");
        
        if (success) {
            if (debugMode) {
                System.out.printf("Device %d: Set direction clockwise command sent successfully%n", deviceID);
            }
            
            try {
                Thread.sleep(100); // Small delay to allow command processing
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            
            isCommandInProgress = false;
            return true;
        }

        isCommandInProgress = false;
        return success;
    }

    /**
     * Sets the position of the encoder to a specific value.
     * 
     * @param position The target position in rotations
     * @return True if the command was sent successfully, false otherwise.
     */
    public boolean setPosition(double position) {
        return setPosition(position, 0.2);
    }

    /**
     * Sets the position of the encoder to a specific value with a custom timeout.
     * 
     * @param position The target position in rotations
     * @param timeout Maximum time to wait for operation completion (seconds)
     * @return True if the command was sent successfully, false otherwise.
     */
    public boolean setPosition(double position, double timeout) {
        if (!waitForReady(1000)) {
            if (debugMode) {
                System.out.printf("Device %d: Encoder not ready for set position command%n", deviceID);
            }
            return false;
        }

        isCommandInProgress = true;
        lastCommandTime = System.currentTimeMillis();

        position = position * CountsPerRevolution;

        boolean success = sendDoubleWithTimeoutCommand(SET_POSITION_API_ID, position, timeout, "Set Position");
        
        if (success) {
            if (debugMode) {
                System.out.printf("Device %d: Set position command sent (Position=%.6f, Timeout=%.3f)%n", 
                                deviceID, position, timeout);
            }
            
            // Wait for the timeout period plus a bit extra
            try {
                Thread.sleep((long)(timeout * 1000) + 100);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            
            isCommandInProgress = false;
            return true;
        }

        isCommandInProgress = false;
        return success;
    }

    /**
     * Resets the encoder to factory default settings.
     * This will clear all user configurations and return the encoder to its initial state.
     * 
     * @return True if the command was sent successfully, false otherwise.
     */
    public boolean resetFactoryDefaults() {
        if (!waitForReady(1000)) {
            if (debugMode) {
                System.out.printf("Device %d: Encoder not ready for factory reset command%n", deviceID);
            }
            return false;
        }

        isCommandInProgress = true;
        lastCommandTime = System.currentTimeMillis();

        boolean success = sendBooleanWithTimeoutCommand(RESET_FACTORY_API_ID, true, COMMAND_TIMEOUT_MS, "Factory Reset");
        
        if (success) {
            if (debugMode) {
                System.out.printf("Device %d: Factory reset command sent successfully%n", deviceID);
            }
            
            // Factory reset may take longer
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            
            FAULT.resetStickyFaults();
            
            isCommandInProgress = false;
            return true;
        }

        isCommandInProgress = false;
        return success;
    }

    /**
     * Checks if a command is currently in progress.
     * 
     * @return True if a command is being executed, false otherwise.
     */
    public boolean isCommandInProgress() {
        checkCommandTimeout(); // Update state if command has timed out
        return isCommandInProgress;
    }

    /**
     * Gets the time elapsed since the last command was sent.
     * 
     * @return Time in milliseconds since last command, or -1 if no command has been sent.
     */
    public long getTimeSinceLastCommand() {
        if (lastCommandTime == 0) {
            return -1;
        }
        return System.currentTimeMillis() - lastCommandTime;
    }

    /**
     * Cancels any command currently in progress.
     * This does not stop the encoder from executing the command, but resets the local state.
     */
    public void cancelCommand() {
        if (isCommandInProgress) {
            if (debugMode) {
                System.out.printf("Device %d: Canceling command in progress%n", deviceID);
            }
            isCommandInProgress = false;
        }
    }

    @Override
    public void close() throws Exception {
        SendableRegistry.remove(this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("CANSense");
        builder.addDoubleProperty("Position", () -> getAbsRotations(), this::setPosition);
    }

    private final CoreDeviceListener positionListener = new CoreDeviceListener() {
        @Override
        public void onDataReceived(byte[] data) {
            synchronized (this) {
                multiTurnCounts = readCanPacketAsLong(data);
                absoluteRotations = (double) multiTurnCounts / CountsPerRevolution;
                relativeRotations = absoluteRotations - Math.floor(absoluteRotations);
            }
        }
    };

    private final CoreDeviceListener velocityAccelListener = new CoreDeviceListener() {
        @Override
        public void onDataReceived(byte[] data) {
            synchronized (this) {
                velocityCPS = readCanPacketAs32BitSigned(data, 0);
                accelerationCPS2 = readCanPacketAs32BitSigned(data, 4);
                velocityRPS = (double) velocityCPS / CountsPerRevolution;
                accelerationRPS2 = (double) accelerationCPS2 / CountsPerRevolution;
            }
        }
    };


}

