package frc.VendorFiles.main.java.com.vertos.encoder;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.CAN;

public class CoreDevice {
    
    // Objects for handling CAN communication and data
    private ScheduledExecutorService scheduler;
    private CAN canDevice;
    private CANData canData;

    private boolean isFaulted = false;

    // Variables and Constants
    private final int deviceID;
    private boolean debugMode;

    private static final int QUERY_API_ID = 1;   // API ID for querying devices
    
    // Command API IDs - these should match the encoder firmware
    protected static final int ZERO_ENCODER_API_ID = 10;        // API ID for zero encoder command
    protected static final int INVERT_DIRECTION_API_ID = 11;    // API ID for invert direction command
    protected static final int SET_POSITION_API_ID = 12;        // API ID for set position command
    protected static final int RESET_FACTORY_API_ID = 13;      // API ID for factory reset command
    protected static final int CLEAR_FAULTS_API_ID = 14;       // API ID for clear faults command

    public CoreDevice(int deviceID, boolean debugMode) {
        this.deviceID = deviceID;
        this.debugMode = debugMode;
        
        // Initialize CAN device with the correct base ID
        this.canDevice = new CAN(deviceID, 17, 10);
        this.canData = new CANData();
        this.scheduler = Executors.newScheduledThreadPool(1);
    }

   /**
    * Starts the periodic reading of encoder data. Runs in a separate thread with a fixed rate of 10 ms
    */
    public void start() {
        scheduler.scheduleAtFixedRate(() -> {
            devicePeriodic(); // Call the method to read encoder data
        }, 0, 10, TimeUnit.MILLISECONDS); // Poll every 10ms
    }

    /**
     * Stops the periodic reading of encoder data.
     * This method should be called when the encoder is no longer needed.
     */
    public void stop() {
        scheduler.shutdown(); // Stop the loop
    }

    public void devicePeriodic() {}

    /**
     * Generic method to attempt a CAN operation and automatically handle fault state.
     * This eliminates the need to manually set isFaulted in every method.
     *
     * @param operation The operation to attempt (lambda function)
     * @param apiID The API ID for error reporting
     * @param <T> The return type of the operation
     * @return The result of the operation, or the provided default value if it fails
     */
    private <T> T attemptCanOperation(CanOperation<T> operation, int apiID, T defaultValue) {
        try {
            T result = operation.execute();
            if (result != null) {
                isFaulted = false; // Success - clear fault state
                return result;
            } else {
                handleCanError(apiID);
                return defaultValue;
            }
        } catch (Exception e) {
            handleCanError(apiID);
            return defaultValue;
        }
    }

    /**
     * Centralized error handling for CAN operations
     */
    private void handleCanError(int apiID) {
        isFaulted = true;
        errorNoCanPacketDebug(apiID);
    }

    /**
     * Functional interface for CAN operations that can fail
     */
    @FunctionalInterface
    private interface CanOperation<T> {
        T execute();
    }

    public long pollCanDeviceLong(int apiID) {
        return attemptCanOperation(() -> {
            if (canDevice.readPacketLatest(apiID, canData)) {
                byte[] receivedData = canData.data;
                if (canData.length >= 8) {
                    return readCanPacketAsLong(receivedData);
                } else {
                    errorByteLengthDebug();
                    return null;
                }
            }
            return null;
        }, apiID, -999L);
    }

    public int pollCanDevice32BitSigned(int apiID, int offset) {
        return attemptCanOperation(() -> {
            if (canDevice.readPacketLatest(apiID, canData)) {
                byte[] receivedData = canData.data;
                if (canData.length >= offset + 4) {
                    return readCanPacketAs32BitSigned(receivedData, offset);
                } else {
                    errorByteLengthDebug();
                    return null;
                }
            }
            return null;
        }, apiID, -999);
    }

    /**
     * Reads a 32-bit signed integer from a CAN packet at the specified offset.
     * This method assumes the data is in big-endian format.
     *
     * @param receivedData The byte array containing the received CAN packet data.
     * @param offset The offset in the byte array where the 32-bit value starts.
     * @return The 32-bit signed integer value.
     */
    private int readCanPacketAs32BitSigned(byte[] receivedData, int offset) {
        // Combine four bytes into a 32-bit signed integer (big-endian)
        int value = ((receivedData[offset] & 0xFF) << 24)
                    | ((receivedData[offset + 1] & 0xFF) << 16)
                    | ((receivedData[offset + 2] & 0xFF) << 8)
                    | (receivedData[offset + 3] & 0xFF);

        return value;
    }


    /**
     * Polls the CAN device for a single byte of data.
     *
     * @param apiID The API ID for the data to read.
     * @return The byte value read from the CAN device, or -1 if no data received.
     */
    public byte pollCanDeviceByte(int apiID) {
        return attemptCanOperation(() -> {
            if (canDevice.readPacketLatest(apiID, canData)) {
                byte[] receivedData = canData.data;
                if (canData.length >= 1) {
                    return receivedData[0];
                } else {
                    errorByteLengthDebug();
                    return null;
                }
            }
            return null;
        }, apiID, (byte) -1);
    }

    //-------------------------------------------------------------------------------------------------------------------
    // Command Sending Methods
    //-------------------------------------------------------------------------------------------------------------------
    
    /**
     * Sends a simple command with no payload to the encoder.
     * 
     * @param apiID The API ID for the command
     * @param commandName The name of the command for debug output
     * @return True if the command was sent successfully, false otherwise
     */
    protected boolean sendSimpleCommand(int apiID, String commandName) {
        byte[] commandData = new byte[1];
        commandData[0] = 0x01; // Simple command trigger byte
        
        boolean success = attemptCanOperation(() -> {
            canDevice.writePacket(commandData, apiID);
            if (debugMode) {
                System.out.printf("Device %d: Sent %s command (API_ID=%d)%n", 
                                deviceID, commandName, apiID);
            }
            return true;
        }, apiID, false);
        
        if (!success && debugMode) {
            System.out.printf("Device %d: Failed to send %s command%n", deviceID, commandName);
        }
        
        return success;
    }
    
    /**
     * Sends a command with a double value (position) to the encoder.
     * 
     * @param apiID The API ID for the command
     * @param value The double value to send
     * @param commandName The name of the command for debug output
     * @return True if the command was sent successfully, false otherwise
     */
    protected boolean sendDoubleCommand(int apiID, double value, String commandName) {
        byte[] commandData = doubleToByteArray(value);
        
        boolean success = attemptCanOperation(() -> {
            canDevice.writePacket(commandData, apiID);
            if (debugMode) {
                System.out.printf("Device %d: Sent %s command (API_ID=%d, Value=%.6f)%n", 
                                deviceID, commandName, apiID, value);
            }
            return true;
        }, apiID, false);
        
        if (!success && debugMode) {
            System.out.printf("Device %d: Failed to send %s command%n", deviceID, commandName);
        }
        
        return success;
    }
    
    /**
     * Sends a command with a timeout parameter.
     * 
     * @param apiID The API ID for the command
     * @param value The double value to send
     * @param timeout The timeout value in seconds
     * @param commandName The name of the command for debug output
     * @return True if the command was sent successfully, false otherwise
     */
    protected boolean sendDoubleWithTimeoutCommand(int apiID, double value, double timeout, String commandName) {
        byte[] commandData = new byte[8];
        
        // Convert double to bytes (first 4 bytes for position, next 4 for timeout)
        byte[] valueBytes = doubleToByteArray(value);
        byte[] timeoutBytes = doubleToByteArray(timeout);
        
        // Pack both values into the command data
        System.arraycopy(valueBytes, 0, commandData, 0, 4);
        System.arraycopy(timeoutBytes, 0, commandData, 4, 4);
        
        boolean success = attemptCanOperation(() -> {
            canDevice.writePacket(commandData, apiID);
            if (debugMode) {
                System.out.printf("Device %d: Sent %s command (API_ID=%d, Value=%.6f, Timeout=%.3f)%n", 
                                deviceID, commandName, apiID, value, timeout);
            }
            return true;
        }, apiID, false);
        
        if (!success && debugMode) {
            System.out.printf("Device %d: Failed to send %s command%n", deviceID, commandName);
        }
        
        return success;
    }
    
    /**
     * Waits for a command acknowledgment from the encoder.
     * 
     * @param apiID The API ID to listen for acknowledgment
     * @param timeoutMs Maximum time to wait for acknowledgment in milliseconds
     * @return True if acknowledgment received, false if timeout
     */
    protected boolean waitForCommandAck(int apiID, long timeoutMs) {
        long startTime = System.currentTimeMillis();
        
        while (System.currentTimeMillis() - startTime < timeoutMs) {
            if (canDevice.readPacketLatest(apiID + 100, canData)) { // Ack API is command API + 100
                if (canData.length >= 1 && canData.data[0] == (byte)0xAC) { // 0xAC = acknowledgment byte
                    if (debugMode) {
                        System.out.printf("Device %d: Received command acknowledgment for API %d%n", 
                                        deviceID, apiID);
                    }
                    return true;
                }
            }
            
            try {
                Thread.sleep(1); // Small delay to prevent busy waiting
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
        
        if (debugMode) {
            System.out.printf("Device %d: Timeout waiting for acknowledgment (API %d)%n", deviceID, apiID);
        }
        return false;
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
        attemptCanOperation(() -> {
            canDevice.writePacket(data, apiID);
            if (debugMode) {
                System.out.printf("Device %d: Sent CAN Packet: API_ID=%d, Data=%s%n",
                        deviceID, apiID, bytesToHex(data));
            }
            return true; // Success
        }, apiID, false);
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
    
    /**
     * Converts a double value to a byte array (IEEE 754 format).
     * 
     * @param value The double value to convert
     * @return A byte array representing the double value
     */
    private byte[] doubleToByteArray(double value) {
        long longBits = Double.doubleToLongBits(value);
        byte[] result = new byte[8];
        
        // Convert to big-endian byte order
        for (int i = 0; i < 8; i++) {
            result[i] = (byte) ((longBits >> (56 - 8 * i)) & 0xFF);
        }
        
        return result;
    }
    
    /**
     * Converts a float value to a 4-byte array (IEEE 754 format).
     * 
     * @param value The float value to convert
     * @return A 4-byte array representing the float value
     */
    private byte[] floatToByteArray(float value) {
        int intBits = Float.floatToIntBits(value);
        byte[] result = new byte[4];
        
        // Convert to big-endian byte order
        result[0] = (byte) ((intBits >> 24) & 0xFF);
        result[1] = (byte) ((intBits >> 16) & 0xFF);
        result[2] = (byte) ((intBits >> 8) & 0xFF);
        result[3] = (byte) (intBits & 0xFF);
        
        return result;
    }

    private void errorNoCanPacketDebug(int apiID) {
        // No CAN packet received
        if (debugMode) {
            System.out.printf("Device %d: Error occurred while reading CAN data for API ID %d.%n", deviceID, apiID);
        }
    }
    

    public boolean getFaulted() {
        return isFaulted;
    }   

    /**
     * Manually clear the fault state. Useful for resetting after resolving issues.
     */
    public void clearFaultState() {
        isFaulted = false;
        if (debugMode) {
            System.out.printf("Device %d: Fault state manually cleared.%n", deviceID);
        }
    }

    private void errorByteLengthDebug() {
        // Not enough data to parse required bytes
        if (debugMode) {
            System.out.printf(
                "Device %d: Received CAN packet with only %d bytes; need more.%n",
                deviceID, canData.length
            );
        }
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
     * Helper method to read fault status from the CAN device.
     *
     * @param apiID The API ID for the fault status.
     * @return True if the fault is detected, false otherwise.
     */
    private boolean readFaultStatus(int apiID) {
        return attemptCanOperation(() -> {
            if (canDevice.readPacketLatest(apiID, canData)) {
                byte[] receivedData = canData.data;
                if (canData.length >= 1) {
                    return (receivedData[0] & 0x01) != 0;
                } else {
                    errorByteLengthDebug();
                    return null;
                }
            }
            return null;
        }, apiID, false);
    }

    /**
     * Reads a CAN packet and converts it to a 64-bit signed long value (native counts).
     * This method assumes the data is in big-endian format.
     *
     * @param receivedData The byte array containing the received CAN packet data.
     * @return The 64-bit signed long value representing the received data.
     */
    private long readCanPacketAsLong(byte[] receivedData) {
        return ((long)(receivedData[0] & 0xFF) << 56)
             | ((long)(receivedData[1] & 0xFF) << 48)
             | ((long)(receivedData[2] & 0xFF) << 40)
             | ((long)(receivedData[3] & 0xFF) << 32)
             | ((long)(receivedData[4] & 0xFF) << 24)
             | ((long)(receivedData[5] & 0xFF) << 16)
             | ((long)(receivedData[6] & 0xFF) << 8)
             | ((long)(receivedData[7] & 0xFF));
    }

    /**
     * Reads a 16-bit signed integer from a CAN packet at the specified offset.
     * This method assumes the data is in big-endian format.
     *
     * @param receivedData The byte array containing the received CAN packet data.
     * @param offset The offset in the byte array where the 16-bit value starts.
     * @return The 16-bit signed integer value.
     */
    private int readCanPacketAs16BitSigned(byte[] receivedData, int offset) {
        // Combine two bytes into a 16-bit signed integer (big-endian)
        int value = ((receivedData[offset] & 0xFF) << 8) | (receivedData[offset + 1] & 0xFF);
        
        // Convert to signed 16-bit value
        if (value > 32767) {
            value -= 65536;
        }
        
        return value;
    }
}