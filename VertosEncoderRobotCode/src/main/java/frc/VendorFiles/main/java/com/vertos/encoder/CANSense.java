package frc.VendorFiles.main.java.com.vertos.encoder;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.hal.CANData;


public class CANSense {
    // Objects for handling CAN communication and data
    private ScheduledExecutorService scheduler;
    private CAN canDevice;
    private CANData canData;

    // Variables and Constants
    private final int deviceID;
    private final boolean debugMode;
    private long multiTurnCounts; // Multi-turn counts for backward compatibility
    private double absoluteRotations;
    private double relativeRotations;
    private double velocityRPS;
    private double accelerationRPS2;  // Added acceleration

    private boolean isStickyFault_Hardware = false;
    private boolean isStickyFault_Undervoltage = false;
    private boolean isStickyFault_BootDuringEnable = false;
    private boolean isStickyFault_BadMagnet = false;
    private boolean isStickyFault_CANGeneral = false;


    // Constants for the encoder
    private final double CountsPerRevolution = 2097152.0; // 2 ^ 21
    
    // Fixed-point scale factor (must match C code)
    private static final double FIXED_POINT_SCALE = 65536.0;
    
    // Default API ID for reading multi-turn counts
    private static final int POSITION_API_ID = 0;  // API ID for position data
    private static final int VELOCITY_API_ID = 16; // API ID for velocity data
    private static final int ACCELERATION_API_ID = 32; // API ID for acceleration data
    private static final int QUERY_API_ID = 1;   // API ID for querying devices
    private static final int HARDWARE_FAULT_API_ID = 48; // API ID for hardware fault
    private static final int UNDERVOLTAGE_FAULT_API_ID = 49; // API ID for undervoltage fault
    private static final int BOOT_DURING_ENABLE_FAULT_API_ID = 50; // API ID for boot during enable fault
    private static final int BAD_MAGNET_FAULT_API_ID = 51; // API ID for magnet fault
    private static final int CAN_GENERAL_FAULT_API_ID = 52; // API ID for general CAN fault

    /**
     * Constructor for CANSense.
     *
     * @param deviceID   The CAN device ID for the encoder.
     * @param debugMode  If true, enables debug output to the console.
     */
    public CANSense(int deviceID, boolean debugMode) {
        this.deviceID = deviceID;
        this.debugMode = debugMode;
        
        // Calculate the base CAN ID to match your C code
        int baseCanId = 0xA080000 + deviceID;  // Must match C code: BASE_ID + device_id
        // Initialize CAN device with the correct base ID
        this.canDevice = new CAN(baseCanId, 8, 10);
        this.canData = new CANData();
        this.scheduler = Executors.newScheduledThreadPool(1);
        start();
    }

    /**
     * The main thread that reads the encoder data.
     * This method is called periodically to read the latest CAN packets for position and velocity.
     */
    private void start() {
        scheduler.scheduleAtFixedRate(() -> {
            encoderThread(); // Call the method to read encoder data
        }, 0, 10, TimeUnit.MILLISECONDS); // Poll every 10ms
    }

    private void stop() {
        scheduler.shutdown(); // Stop the loop
    }

    public void encoderThread() {
        
        // Read absolute rotations (8 bytes)
        if (canDevice.readPacketLatest((POSITION_API_ID), canData)) {
            byte[] receivedData = canData.data;
        
            if (canData.length >= 8) {
                long absoluteBits = readCanPacket(receivedData);
                this.absoluteRotations = Double.longBitsToDouble(absoluteBits);
                // Calculate relative rotations from absolute (0.0 to 1.0)
                this.relativeRotations = absoluteRotations - Math.floor(absoluteRotations);
                // Calculate equivalent multi-turn counts for backward compatibility
                multiTurnCounts = (long)(absoluteRotations * CountsPerRevolution);
        
                if (debugMode) {
                    System.out.printf(
                        "Device %d: absoluteRotations = %.12f, relativeRotations = %.12f%n",
                        deviceID, absoluteRotations, relativeRotations
                    );
                }
            }
        }

        // Read velocity (8 bytes)  
        if (canDevice.readPacketLatest((VELOCITY_API_ID), canData)) {
            byte[] receivedData = canData.data;
            if (canData.length >= 8) {
                long velocityBits = readCanPacket(receivedData);
                velocityRPS = Double.longBitsToDouble(velocityBits);

                if (debugMode) {
                    System.out.printf(
                        "Device %d: velocity = %.12f RPS%n",
                        deviceID, velocityRPS
                    );
                }
            }
        } else {
            // Can Not found
            errorNoCanPacketDebug(VELOCITY_API_ID);
        }


        // Read Accel (8 bytes)  
        if (canDevice.readPacketLatest((ACCELERATION_API_ID), canData)) {
            byte[] receivedData = canData.data;

            if (canData.length >= 8) {
                // Extract 64-bit velocity double
                long accelBits = readCanPacket(receivedData);
                // Convert bits to double
                accelerationRPS2 = Double.longBitsToDouble(accelBits);
                if (debugMode) {
                    System.out.printf(
                        "Device %d: acceleration = %.12f RPS%n",
                        deviceID, accelerationRPS2
                    );
                }
            }
        } else {
            // Can Not found
            errorNoCanPacketDebug(ACCELERATION_API_ID);
        }

    }


    //---------------------------------------------------------------------------------------
    // Getters for the encoder data
    //---------------------------------------------------------------------------------------
    /**
     * Returns the multi-turn counts for backward compatibility.
     * This is calculated as absolute rotations multiplied by CountsPerRevolution.
     *
     * @return The multi-turn counts as a long.
     */
    public double getAbsRotations() {
        return absoluteRotations;
    }

    /**
     * Returns the relative rotations (0.0 to 1.0) based on absolute rotations.
     * This is calculated as absolute rotations minus the floor of absolute rotations.
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

    public double getInputVoltage() {
        // Placeholder for supply voltage, not implemented
        return 0.0;
    }

    public double getMagnetHealth() {
        return 0.0; // Placeholder for magnet health, not implemented
    }

    /**
     * Determines whether the CAN device is connected by sending and receiving packets.
     *
     * @return True if the device responds to a query, false otherwise.
     */
    public boolean isConnected() {
        // Send a query packet to the device
        byte[] queryData = new byte[8]; // Example query data, can be adjusted as needed
        sendPacket(QUERY_API_ID, queryData);

        // Attempt to read a response packet
        boolean packetReceived = canDevice.readPacketNew(QUERY_API_ID, canData);

        if (debugMode) {
            if (packetReceived) {
                System.out.printf("Device %d: Connection verified via CAN packet response.%n", deviceID);
            } else {
                System.out.printf("Device %d: No response received; device may not be connected.%n", deviceID);
            }
        }

        return packetReceived;
    }

    /**
     * Retrieves the current hardware fault status.
     *
     * @return True if a hardware fault is detected, false otherwise.
     */
    public boolean getFault_Hardware() {
        boolean isFault = readFaultStatus(HARDWARE_FAULT_API_ID);
        // Update sticky fault status if a hardware fault is detected
        if(!isStickyFault_Hardware) {
            isStickyFault_Hardware = isFault;
        }
        return isFault;
    }

    /**
     * Retrieves the current undervoltage fault status.
     *
     * @return True if an undervoltage fault is detected, false otherwise.
     */
    public boolean getFault_Undervoltage() {
        boolean isFault = readFaultStatus(UNDERVOLTAGE_FAULT_API_ID);
        if (!isStickyFault_Undervoltage) {
            isStickyFault_Undervoltage = isFault;
        }
        return isFault;
    }

    /**
     * Retrieves the current boot during enable fault status.
     *
     * @return True if a boot during enable fault is detected, false otherwise.
     */
    public boolean getFault_BootDuringEnable() {
        boolean isFault = readFaultStatus(BOOT_DURING_ENABLE_FAULT_API_ID);
        if (!isStickyFault_BootDuringEnable) {
            isStickyFault_BootDuringEnable = isFault;
        }
        return isFault;
    }

    /**
     * Retrieves the current magnet fault status.
     *
     * @return True if a magnet fault is detected, false otherwise.
     */
    public boolean getFault_BadMagnet() {
        boolean isFault = readFaultStatus(BAD_MAGNET_FAULT_API_ID);
        if (!isStickyFault_BadMagnet) {
            isStickyFault_BadMagnet = isFault;
        }
        return isFault;
    }

    /**
     * Retrieves the current general CAN fault status.
     *
     * @return True if a general CAN fault is detected, false otherwise.
     */
    public boolean getFault_CANGeneral() {
        boolean isFault = readFaultStatus(CAN_GENERAL_FAULT_API_ID);
        if (!isStickyFault_CANGeneral) {
            isStickyFault_CANGeneral = isFault;
        }
        return isFault;
    }

    /**
     * Retrieves the sticky hardware fault status.
     *
     * @return True if a sticky hardware fault has been detected, false otherwise.
     */
    public boolean getStickyFault_Hardware() {
        return isStickyFault_Hardware;
    }

    /**
     * Retrieves the sticky undervoltage fault status.
     *
     * @return True if a sticky undervoltage fault has been detected, false otherwise.
     */
    public boolean getStickyFault_Undervoltage() {
        return isStickyFault_Undervoltage;
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
     *
     * @return True if a sticky general CAN fault has been detected, false otherwise.
     */
    public boolean getStickyFault_CANGeneral() {
        return isStickyFault_CANGeneral;
    }


    /**
     * Resets all sticky fault flags to false.
     * This method allows the user to clear all sticky fault statuses.
     */
    public void resetStickyFaults() {
        isStickyFault_Hardware = false;
        isStickyFault_Undervoltage = false;
        isStickyFault_BootDuringEnable = false;
        isStickyFault_BadMagnet = false;

        if (debugMode) {
            System.out.println("All sticky faults have been reset.");
        }
    }

    /**
     * Helper method to read fault status from the CAN device.
     *
     * @param apiID The API ID for the fault status.
     * @return True if the fault is detected, false otherwise.
     */
    private boolean readFaultStatus(int apiID) {
        if (canDevice.readPacketLatest(apiID, canData)) {
            byte[] receivedData = canData.data;
            if (canData.length >= 1) {
                return (receivedData[0] & 0x01) != 0; // Check the first byte for fault status
            } else {
                errorByteLengthDebug();
            }
        } else {
            errorNoCanPacketDebug(apiID);
        }
        return false;
    }

    //-------------------------------------------------------------------------------------------
    // Senders
    //-------------------------------------------------------------------------------------------
    /**
     * Sends a command to zero the encoder.
     */
    public void zeroEncoder() {
        // Send a command to zero the encoder
        // This method can be implemented based on specific requirements
        // For now, it does nothing
        if (debugMode) {
            System.out.printf("Device %d: Zeroing encoder (not implemented).%n", deviceID);
        }
    }

    /**
     * Inverts the direction of the encoder.
     */
    public void invertDirection() {
        // Invert the direction of the encoder
        // This method can be implemented based on specific requirements
        // For now, it does nothing
        if (debugMode) {
            System.out.printf("Device %d: Inverting direction (not implemented).%n", deviceID);
        }
    }

    /**
     * Sets the position of the encoder.
     * @param position
     */
    public void setPosition(double position) {
        setPosition(position, 0.2);
    }

    public void setPosition(double position, double timeout) {
        // Set the position of the encoder
        // This method can be implemented based on specific requirements
        // For now, it does nothing
        if (debugMode) {
            System.out.printf("Device %d: Setting position to %d (not implemented).%n", deviceID, position);
        }
    }

    public void resetFactoryDefaults() {
        // Reset the encoder to factory defaults
        // This method can be implemented based on specific requirements
        // For now, it does nothing
        if (debugMode) {
            System.out.printf("Device %d: Resetting to factory defaults (not implemented).%n", deviceID);
        }
    }

    //-------------------------------------------------------------------------------------------------------------------
    // Helpers
    //-------------------------------------------------------------------------------------------------------------------
    /**
     * Sends a CAN packet to the device.
     *
     * @param apiID The API ID for the packet.
     * @param data  The payload to send.
     */
    public void sendPacket(int apiID, byte[] data) {
        try {
            canDevice.writePacket(data, apiID);
            if (debugMode) {
                System.out.printf("Device %d: Sent CAN Packet: API_ID=%d, Data=%s%n",
                        deviceID, apiID, bytesToHex(data));
            }
        } catch (Exception e) {
            errorNoCanPacketDebug(apiID);
        }
    }

    /**
     * Reads a CAN packet and converts it to a 64-bit long value.
     * This method assumes the data is in big-endian format.
     *
     * @param receivedData The byte array containing the received CAN packet data.
     * @return The 64-bit long value representing the received data.
     */
    private long readCanPacket(byte[] receivedData) {
        long recievedBits = ((long)(receivedData[0] & 0xFF) << 56)
                        | ((long)(receivedData[1] & 0xFF) << 48)
                        | ((long)(receivedData[2] & 0xFF) << 40)
                        | ((long)(receivedData[3] & 0xFF) << 32)
                        | ((long)(receivedData[4] & 0xFF) << 24)
                        | ((long)(receivedData[5] & 0xFF) << 16)
                        | ((long)(receivedData[6] & 0xFF) << 8)
                        | ((long)(receivedData[7] & 0xFF));
        return recievedBits;
    }

    /**
     * Method to query devices using API ID 1.
     * Continuously reads CAN packets with a timeout of 1000 ms until no new packets arrive.
     * Stores all received data in an array and prints the received devices.
     *
     * @return List of received CAN packet data as byte arrays.
     */
    public List<byte[]> queryDevices() {
        List<byte[]> receivedPackets = new ArrayList<>();
        boolean packetReceived;
        int responseCount = 0;

        do {
            packetReceived = canDevice.readPacketNew(QUERY_API_ID, canData);
            if (packetReceived) {
                responseCount++;
                byte[] dataCopy = new byte[canData.length];
                System.arraycopy(canData.data, 0, dataCopy, 0, canData.length);
                receivedPackets.add(dataCopy);
                System.out.printf("Response %d: Received CAN Packet: Data=%s%n", responseCount, bytesToHex(dataCopy));
            }
        } while (packetReceived);

        System.out.printf("Total number of responses: %d%n", responseCount);

        if (receivedPackets.isEmpty()) {
            System.out.println("No CAN packets received during device query.");
        }

        return receivedPackets;
    }

    /**
     * Helper function to convert a byte array to a hexadecimal string.
     *
     * @param bytes The byte array to convert.
     * @return A string representation of the byte array in hexadecimal format.
     */
    private String bytesToHex(byte[] bytes) {
        StringBuilder sb = new StringBuilder();
        for (byte b : bytes) {
            sb.append(String.format("%02X ", b));
        }
        return sb.toString().trim();
    }

    private void errorNoCanPacketDebug(int apiID) {
        // No CAN packet received
        if (debugMode) {
            System.out.printf("Device %d: Error occurred while reading CAN data for API ID %d.%n", deviceID, apiID);
        }
    }

    private void errorByteLengthDebug() {
        // Not enough data to parse 64 bits
        if (debugMode) {
            System.out.printf(
                "Device %d: Received CAN packet with only %d bytes; need 8.%n",
                deviceID, canData.length
            );
        }
    }
}