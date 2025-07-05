package frc.VendorFiles.main.java.com.vertos.encoder;

public class ParentCANSense extends CoreDevice{

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
    
    // Fault status variables (kept for backward compatibility)
    private boolean isStickyFault_Hardware = false; 
    private boolean isStickyFault_BootDuringEnable = false;
    private boolean isStickyFault_LoopOverrun = false;
    private boolean isStickyFault_BadMagnet = false;
    private boolean isStickyFault_CANGeneral = false;
    private boolean isStickyFault_MomentaryCanBusLoss = false; 
    private boolean isStickyFault_CANClogged = false;
    private boolean isStickyFault_RotationOverspeed = false;
    private boolean isStickyFault_UnderVolted = false;


    // Constants for the encoder
    private final double CountsPerRevolution = 2097152.0; // 2 ^ 21
    
    // API IDs for reading different data types (must match C code)
    private static final int POSITION_API_ID = 0;          // API ID for position data (8 bytes)
    private static final int VELOCITY_ACCEL_API_ID = 16;    // API ID for combined velocity and acceleration data (4 bytes)
    private static final int FAULT_API_ID = 32;            // API ID for hardware fault (legacy)

    // Command execution state tracking
    private boolean isCommandInProgress = false;
    private long lastCommandTime = 0;
    private static final long COMMAND_TIMEOUT_MS = 5000; // 5 second timeout for commands
    private static final int VELOCITY_SCALE_FACTOR = 10;      // Must match C code
    private static final int ACCELERATION_SCALE_FACTOR = 100; 

    /**
     * Constructor for CANSense.
     *
     * @param deviceID   The CAN device ID for the encoder.
     * @param debugMode  If true, enables debug output to the console.
     */
    public ParentCANSense(int deviceID, boolean debugMode) {
        super(deviceID, debugMode);
        this.deviceID = deviceID;
        this.debugMode = debugMode;
    }

    @Override
    public void devicePeriodic() {
        
        // Read position data
        this.multiTurnCounts = pollCanDeviceLong(POSITION_API_ID);
        // Calculate rotations from native counts
        this.absoluteRotations = (double)multiTurnCounts / CountsPerRevolution;
        this.relativeRotations = absoluteRotations - Math.floor(absoluteRotations);

        // Read velocity and acceleration data and apply scaling directly
        this.velocityCPS = pollCanDevice32BitSigned(VELOCITY_ACCEL_API_ID, 0) << 11;
        this.accelerationCPS2 = pollCanDevice32BitSigned(VELOCITY_ACCEL_API_ID, 4) << 16 ;

        // Calculate RPS from counts per second
        this.velocityRPS = (double)velocityCPS / CountsPerRevolution;
        // Calculate RPS² from counts per second squared
        this.accelerationRPS2 = (double)accelerationCPS2 / CountsPerRevolution;
        
        // Read boolean status data
        byte booleanStatusByte = pollCanDeviceByte(FAULT_API_ID);

        // Unpack the 8 boolean values from the single byte and make them sticky
        this.isStickyFault_Hardware          |= (booleanStatusByte & 0x01) != 0; // Bit 0: errorStatus
        this.isStickyFault_LoopOverrun       |= (booleanStatusByte & 0x02) != 0; // Bit 1: encoder init status
        this.isStickyFault_CANGeneral        |= (booleanStatusByte & 0x04) != 0; // Bit 2: CAN status
        this.isStickyFault_BadMagnet         |= (booleanStatusByte & 0x08) != 0; // Bit 3: sensor calibration
        this.isStickyFault_MomentaryCanBusLoss |= (booleanStatusByte & 0x10) != 0; // Bit 4: proximity sensor
        this.isStickyFault_RotationOverspeed |= (booleanStatusByte & 0x20) != 0; // Bit 5: flash memory
        this.isStickyFault_CANClogged        |= (booleanStatusByte & 0x40) != 0; // Bit 6: USB connection
        this.isStickyFault_UnderVolted       |= (booleanStatusByte & 0x80) != 0; // Bit 7: system ready
        
        
        // Check if any commands have timed out
        checkCommandTimeout();
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

    //----------------------------------------------------------------------------------------
    // Getters for native encoder data (UPDATED)
    //----------------------------------------------------------------------------------------
    
    /**
     * Returns the raw multi-turn encoder counts.
     *
     * @return The multi-turn counts as a long (native encoder units).
     */
    public long getMultiTurnCounts() {
        return multiTurnCounts;
    }
    
    /**
     * Returns the single-turn encoder counts (0 to CountsPerRevolution-1).
     *
     * @return The single-turn counts as a long.
     */
    public long getSingleTurnCounts() {
        return multiTurnCounts % (long)CountsPerRevolution;
    }
    
    /**
     * Returns the velocity in counts per second.
     *
     * @return The velocity in counts per second (now 32-bit).
     */
    public int getVelocityCPS() {
        return velocityCPS;
    }
    
    /**
     * Returns the acceleration in counts per second squared.
     *
     * @return The acceleration in counts per second squared (now 32-bit).
     */
    public int getAccelerationCPS2() {
        return accelerationCPS2;
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


    /**
     * Retrieves the sticky hardware fault status.
     * (Updated to use boolean status data)
     *
     * @return True if a sticky hardware fault has been detected, false otherwise.
     */
    public boolean getStickyFault_Hardware() {
        return isStickyFault_Hardware;
    }

    /**
     * Retrieves the sticky boot during enable fault status.
     *
     * @return True if a sticky boot during enable fault has been detected, false otherwise.
     */
    public boolean getStickyFault_BootDuringEnable() {
        return isStickyFault_BootDuringEnable;
    }

    /**
     * Retrieves the sticky magnet fault status.
     *
     * @return True if a sticky magnet fault has been detected, false otherwise.
     */
    public boolean getStickyFault_BadMagnet() {
        return isStickyFault_BadMagnet;
    }

    /**
     * Retrieves the sticky general CAN fault status.
     * (Updated to use boolean status data)
     *
     * @return True if a sticky general CAN fault has been detected, false otherwise.
     */
    public boolean getStickyFault_CANGeneral() {
        return isStickyFault_CANGeneral;
    }

    /**
     * Retrieves the sticky loop overrun fault status.
     *
     * @return True if a sticky loop overrun fault has been detected, false otherwise.
     */
    public boolean getStickyFault_LoopOverrun() {
        return isStickyFault_LoopOverrun;
    }

    /**
     * Retrieves the sticky momentary CAN bus loss fault status.
     *
     * @return True if a sticky momentary CAN bus loss fault has been detected, false otherwise.
     */
    public boolean getStickyFault_MomentaryCanBusLoss() {
        return isStickyFault_MomentaryCanBusLoss;
    }

    /**
     * Retrieves the sticky CAN clogged fault status.
     *
     * @return True if a sticky CAN clogged fault has been detected, false otherwise.
     */
    public boolean getStickyFault_CANClogged() {
        return isStickyFault_CANClogged;
    }

    /**
     * Retrieves the sticky rotation overspeed fault status.
     *
     * @return True if a sticky rotation overspeed fault has been detected, false otherwise.
     */
    public boolean getStickyFault_RotationOverspeed() {
        return isStickyFault_RotationOverspeed;
    }

    /**
     * Retrieves the sticky under-volted fault status.
     *
     * @return True if a sticky under-volted fault has been detected, false otherwise.
     */
    public boolean getStickyFault_UnderVolted() {
        return isStickyFault_UnderVolted;
    }

    /**
     * Resets all sticky fault flags to false.
     * This method allows the user to clear all sticky fault statuses.
     */
    public void resetStickyFaults() {
        // Send clear faults command to encoder
        if (!waitForReady(1000)) {
            if (debugMode) {
                System.out.printf("Device %d: Encoder not ready for clear faults command%n", deviceID);
            }
            return;
        }

        isCommandInProgress = true;
        lastCommandTime = System.currentTimeMillis();

        boolean success = sendSimpleCommand(CLEAR_FAULTS_API_ID, "Clear Faults");
        
        if (success) {
            // Clear local fault states
            isStickyFault_Hardware = false;
            isStickyFault_CANGeneral = false;
            isStickyFault_LoopOverrun = false;
            isStickyFault_MomentaryCanBusLoss = false;
            isStickyFault_BootDuringEnable = false;
            isStickyFault_BadMagnet = false;

            if (debugMode) {
                System.out.printf("Device %d: All sticky faults have been reset.%n", deviceID);
            }
            
            // Wait for command acknowledgment
            if (waitForCommandAck(CLEAR_FAULTS_API_ID, 1000)) {
                if (debugMode) {
                    System.out.printf("Device %d: Clear faults command acknowledged%n", deviceID);
                }
            }
        } else {
            if (debugMode) {
                System.out.printf("Device %d: Failed to send clear faults command%n", deviceID);
            }
        }

        isCommandInProgress = false;
    }

    //-------------------------------------------------------------------------------------------
    // Senders - IMPLEMENTED WITH ACTUAL CAN COMMUNICATION
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

        boolean success = sendSimpleCommand(ZERO_ENCODER_API_ID, "Zero Encoder");
        
        if (success) {
            if (debugMode) {
                System.out.printf("Device %d: Zero encoder command sent successfully%n", deviceID);
            }
            
            // Wait for command acknowledgment
            if (waitForCommandAck(ZERO_ENCODER_API_ID, 2000)) {
                if (debugMode) {
                    System.out.printf("Device %d: Zero encoder command acknowledged%n", deviceID);
                }
                isCommandInProgress = false;
                return true;
            } else {
                if (debugMode) {
                    System.out.printf("Device %d: Zero encoder command timeout%n", deviceID);
                }
            }
        }

        isCommandInProgress = false;
        return success;
    }

    /**
     * Inverts the direction of the encoder.
     * Changes the sign of position, velocity, and acceleration readings.
     * 
     * @return True if the command was sent successfully, false otherwise.
     */
    public boolean invertDirection() {
        if (!waitForReady(1000)) {
            if (debugMode) {
                System.out.printf("Device %d: Encoder not ready for invert direction command%n", deviceID);
            }
            return false;
        }

        isCommandInProgress = true;
        lastCommandTime = System.currentTimeMillis();

        boolean success = sendSimpleCommand(INVERT_DIRECTION_API_ID, "Invert Direction");
        
        if (success) {
            if (debugMode) {
                System.out.printf("Device %d: Invert direction command sent successfully%n", deviceID);
            }
            
            // Wait for command acknowledgment
            if (waitForCommandAck(INVERT_DIRECTION_API_ID, 2000)) {
                if (debugMode) {
                    System.out.printf("Device %d: Invert direction command acknowledged%n", deviceID);
                }
                isCommandInProgress = false;
                return true;
            } else {
                if (debugMode) {
                    System.out.printf("Device %d: Invert direction command timeout%n", deviceID);
                }
            }
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

        boolean success = sendDoubleWithTimeoutCommand(SET_POSITION_API_ID, position, timeout, "Set Position");
        
        if (success) {
            if (debugMode) {
                System.out.printf("Device %d: Set position command sent (Position=%.6f, Timeout=%.3f)%n", 
                                deviceID, position, timeout);
            }
            
            // Wait for command acknowledgment (use timeout parameter converted to ms)
            long timeoutMs = Math.max(1000, (long)(timeout * 1000) + 1000); // At least 1 second, plus command timeout
            if (waitForCommandAck(SET_POSITION_API_ID, timeoutMs)) {
                if (debugMode) {
                    System.out.printf("Device %d: Set position command acknowledged%n", deviceID);
                }
                isCommandInProgress = false;
                return true;
            } else {
                if (debugMode) {
                    System.out.printf("Device %d: Set position command timeout%n", deviceID);
                }
            }
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

        boolean success = sendSimpleCommand(RESET_FACTORY_API_ID, "Reset Factory Defaults");
        
        if (success) {
            if (debugMode) {
                System.out.printf("Device %d: Factory reset command sent successfully%n", deviceID);
            }
            
            // Factory reset may take longer, so use extended timeout
            if (waitForCommandAck(RESET_FACTORY_API_ID, 5000)) {
                if (debugMode) {
                    System.out.printf("Device %d: Factory reset command acknowledged%n", deviceID);
                }
                
                // Clear local fault states after factory reset
                isStickyFault_Hardware = false;
                isStickyFault_CANGeneral = false;
                isStickyFault_LoopOverrun = false;
                isStickyFault_MomentaryCanBusLoss = false;
                isStickyFault_BootDuringEnable = false;
                isStickyFault_BadMagnet = false;
                
                isCommandInProgress = false;
                return true;
            } else {
                if (debugMode) {
                    System.out.printf("Device %d: Factory reset command timeout%n", deviceID);
                }
            }
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


}