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

    // Non-sticky fault status variables
    private boolean isFault_Hardware = false; 
    private boolean isFault_BootDuringEnable = false;
    private boolean isFault_LoopOverrun = false;
    private boolean isFault_BadMagnet = false;
    private boolean isFault_CANGeneral = false;
    private boolean isFault_MomentaryCanBusLoss = false; 
    private boolean isFault_CANClogged = false;
    private boolean isFault_RotationOverspeed = false;
    private boolean isFault_UnderVolted = false;

    // Constants for the encoder
    private final double CountsPerRevolution = 2097152.0; // 2 ^ 21
    
    // API IDs for reading different data types (must match C code)
    private static final int POSITION_API_ID = 0;          // API ID for position data (8 bytes)
    private static final int VELOCITY_ACCEL_API_ID = 16;    // API ID for combined velocity and acceleration data (4 bytes)
    private static final int FAULT_API_ID = 32;            // API ID for hardware fault (legacy)

    // Command API IDs - these should match the encoder firmware
    private static final int ZERO_ENCODER_API_ID = 10;        // API ID for zero encoder command
    private static final int SET_DIRECTION_CLOCKWISE_API_ID = 11;    // API ID for set direction command (same as invert direction)
    private static final int SET_DIRECTION_COUNTER_CLOCKWISE_API_ID = 12; // API ID for set direction counter-clockwise command
    private static final int SET_POSITION_API_ID = 14;        // API ID for set position command
    private static final int RESET_FACTORY_API_ID = 15;      // API ID for factory reset command


    // Command execution state tracking
    private boolean isCommandInProgress = false;
    private long lastCommandTime = 0;
    private static final long COMMAND_TIMEOUT_MS = 5000; // 5 second timeout for commands



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
        addListener(positionListener, POSITION_API_ID);
        addListener(velocityAccelListener, VELOCITY_ACCEL_API_ID);
        addListener(faultListener, FAULT_API_ID);
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
     * Resets the sticky hardware fault status.
     */
    public void resetStickyFault_Hardware() {
        isStickyFault_Hardware = false;
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
     * Resets the sticky boot during enable fault status.
     */
    public void resetStickyFault_BootDuringEnable() {
        isStickyFault_BootDuringEnable = false;
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
     * Resets the sticky magnet fault status.
     */
    public void resetStickyFault_BadMagnet() {
        isStickyFault_BadMagnet = false;
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
     * Resets the sticky general CAN fault status.
     */
    public void resetStickyFault_CANGeneral() {
        isStickyFault_CANGeneral = false;
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
     * Resets the sticky loop overrun fault status.
     */
    public void resetStickyFault_LoopOverrun() {
        isStickyFault_LoopOverrun = false;
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
     * Resets the sticky momentary CAN bus loss fault status.
     */
    public void resetStickyFault_MomentaryCanBusLoss() {
        isStickyFault_MomentaryCanBusLoss = false;
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
     * Resets the sticky CAN clogged fault status.
     */
    public void resetStickyFault_CANClogged() {
        isStickyFault_CANClogged = false;
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
     * Resets the sticky rotation overspeed fault status.
     */
    public void resetStickyFault_RotationOverspeed() {
        isStickyFault_RotationOverspeed = false;
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
     * Resets the sticky under-volted fault status.
     */
    public void resetStickyFault_UnderVolted() {
        isStickyFault_UnderVolted = false;
    }

    /**
     * Retrieves the hardware fault status.
     *
     * @return True if a hardware fault has been detected, false otherwise.
     */
    public boolean getFault_Hardware() {
        return isFault_Hardware;
    }

    /**
     * Retrieves the boot during enable fault status.
     *
     * @return True if a boot during enable fault has been detected, false otherwise.
     */
    public boolean getFault_BootDuringEnable() {
        return isFault_BootDuringEnable;
    }

    /**
     * Retrieves the loop overrun fault status.
     *
     * @return True if a loop overrun fault has been detected, false otherwise.
     */
    public boolean getFault_LoopOverrun() {
        return isFault_LoopOverrun;
    }

    /**
     * Retrieves the bad magnet fault status.
     *
     * @return True if a bad magnet fault has been detected, false otherwise.
     */
    public boolean getFault_BadMagnet() {
        return isFault_BadMagnet;
    }

    /**
     * Retrieves the general CAN fault status.
     *
     * @return True if a general CAN fault has been detected, false otherwise.
     */
    public boolean getFault_CANGeneral() {
        return isFault_CANGeneral;
    }

    /**
     * Retrieves the momentary CAN bus loss fault status.
     *
     * @return True if a momentary CAN bus loss fault has been detected, false otherwise.
     */
    public boolean getFault_MomentaryCanBusLoss() {
        return isFault_MomentaryCanBusLoss;
    }

    /**
     * Retrieves the CAN clogged fault status.
     *
     * @return True if a CAN clogged fault has been detected, false otherwise.
     */
    public boolean getFault_CANClogged() {
        return isFault_CANClogged;
    }

    /**
     * Retrieves the rotation overspeed fault status.
     *
     * @return True if a rotation overspeed fault has been detected, false otherwise.
     */
    public boolean getFault_RotationOverspeed() {
        return isFault_RotationOverspeed;
    }

    /**
     * Retrieves the under-volted fault status.
     *
     * @return True if an under-volted fault has been detected, false otherwise.
     */
    public boolean getFault_UnderVolted() {
        return isFault_UnderVolted;
    }


    /**
     * Resets all sticky fault flags to false.
     * This method allows the user to clear all sticky fault statuses.
     */
    public void resetStickyFaults() {
        // Clear local fault states
        isStickyFault_Hardware = false;
        isStickyFault_CANGeneral = false;
        isStickyFault_LoopOverrun = false;
        isStickyFault_MomentaryCanBusLoss = false;
        isStickyFault_BootDuringEnable = false;
        isStickyFault_BadMagnet = false;
        isStickyFault_CANClogged = false;
        isStickyFault_RotationOverspeed = false;
        isStickyFault_UnderVolted = false;

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

        boolean success = sendSimpleCommand(SET_DIRECTION_CLOCKWISE_API_ID, "Set Direction Counter-Clockwise");
        
        if (success) {
            if (debugMode) {
                System.out.printf("Device %d: Set direction counter-clockwise command sent successfully%n", deviceID);
            }
            
            // Wait for command acknowledgment
            if (waitForCommandAck(SET_DIRECTION_COUNTER_CLOCKWISE_API_ID, 2000)) {
                if (debugMode) {
                    System.out.printf("Device %d: Set direction counter-clockwise command acknowledged%n", deviceID);
                }
                isCommandInProgress = false;
                return true;
            } else {
                if (debugMode) {
                    System.out.printf("Device %d: Set direction counter-clockwise command timeout%n", deviceID);
                }
            }
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

        boolean success = sendSimpleCommand(SET_DIRECTION_CLOCKWISE_API_ID, "Set Direction Clockwise");
        
        if (success) {
            if (debugMode) {
                System.out.printf("Device %d: Set direction clockwise command sent successfully%n", deviceID);
            }
            
            // Wait for command acknowledgment
            if (waitForCommandAck(SET_DIRECTION_CLOCKWISE_API_ID, 2000)) {
                if (debugMode) {
                    System.out.printf("Device %d: Set direction clockwise command acknowledged%n", deviceID);
                }
                isCommandInProgress = false;
                return true;
            } else {
                if (debugMode) {
                    System.out.printf("Device %d: Set direction clockwise command timeout%n", deviceID);
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

    private boolean previousCANFault = false;

    private final CoreDeviceListener faultListener = new CoreDeviceListener() {
        @Override
        public void onDataReceived(byte[] data) {

            synchronized (this) {
                byte booleanStatusByte = data[0];
                isFault_Hardware              = (booleanStatusByte & 0x01) != 0;  // Bit 0
                isFault_LoopOverrun           = (booleanStatusByte & 0x02) != 0;  // Bit 1
                isFault_CANGeneral            = (booleanStatusByte & 0x04) != 0;  // Bit 2: isCANInvalid
                isFault_BootDuringEnable      = (booleanStatusByte & 0x08) != 0;  // Bit 3: isResetDuringEnable
                isFault_BadMagnet             = (booleanStatusByte & 0x10) != 0;  // Bit 4: isMagnetWeakSignal
                isFault_RotationOverspeed     = (booleanStatusByte & 0x20) != 0;  // Bit 5
                isFault_CANClogged            = (booleanStatusByte & 0x40) != 0;  // Bit 6
                isFault_UnderVolted           = (booleanStatusByte & 0x80) != 0;  // Bit 7

                isStickyFault_BootDuringEnable      |= isFault_BootDuringEnable;
                isStickyFault_Hardware              |= isFault_Hardware;
                isStickyFault_LoopOverrun           |= isFault_LoopOverrun;
                isStickyFault_CANGeneral            |= isFault_CANGeneral;
                isStickyFault_BadMagnet             |= isFault_BadMagnet;
                isStickyFault_RotationOverspeed     |= isFault_RotationOverspeed;
                isStickyFault_CANClogged            |= isFault_CANClogged;
                isStickyFault_UnderVolted           |= isFault_UnderVolted;

                boolean currentCANFault = isFault_CANClogged || isFault_CANGeneral;

                if (previousCANFault && !currentCANFault) {
                    isStickyFault_MomentaryCanBusLoss = true;
                }

                previousCANFault = currentCANFault;
            }
        }
    };


}