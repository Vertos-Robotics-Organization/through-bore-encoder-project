package frc.VendorFiles.main.java.com.vertos.encoder;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.CAN;

public class CoreDevice {
    
    // Objects for handling CAN communication and data
    private static final ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(1);

    private CAN canDevice;
    private CANData canData;
    private final List<Pair<CoreDeviceListener, Integer>> listenerPairs = new ArrayList<>();

    private boolean isFaulted = false;

    private final int BASE_ID = 0xA110000;

    // Variables and Constants
    private final int deviceID;
    private boolean debugMode;

    private static final int QUERY_API_ID = 1;   // API ID for querying devices
    

    public CoreDevice(int deviceID, boolean debugMode) {
        this.deviceID = deviceID;
        this.debugMode = debugMode;
        
        // Initialize CAN device with the correct base ID + device ID
        // This should match how the C code calculates identifiers
        this.canDevice = new CAN(deviceID, 17, 10);
        this.canData = new CANData();
        start();
    }

    /**
     * Registers a listener for device events.
     */
    protected void addListener(CoreDeviceListener listener, int apiID) {
        listenerPairs.add(new Pair<>(listener, apiID));
    }

    /**
     * Removes a listener for device events.
     */
    protected void removeListener(CoreDeviceListener listener, int apiID) {
        listenerPairs.removeIf(pair -> Objects.equals(pair.getListener(), listener) && Objects.equals(pair.getAPIID(), apiID));
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

    /**
     * Calculate the correct CAN identifier for receiving data
     * This matches the C code: baseCanId | (apiId << 10)
     */
    private int getRxIdentifier(int apiId) {
        return (BASE_ID + deviceID) | (apiId << 10);
    }

    /**
     * Calculate the correct CAN identifier for sending commands
     * This should match what the C code expects to receive
     */
    private int getTxIdentifier(int apiId) {
        return (BASE_ID + deviceID) | (apiId << 10);
    }

    /**
     * Periodic method to read data from the CAN device.
     * This method checks for new CAN packets and notifies listeners.
     */
    public void devicePeriodic() {
        for (Pair<CoreDeviceListener, Integer> listenerPair : listenerPairs) {
            try {
                
                if (canDevice.readPacketLatest(listenerPair.getAPIID(), canData)) {
                    // Validate packet before processing
                    if (canData.data != null && canData.length > 0) {
                        byte[] receivedData = new byte[canData.length];
                        System.arraycopy(canData.data, 0, receivedData, 0, canData.length);
                        listenerPair.getListener().onDataReceived(receivedData);
                    }
                }
            } catch (Exception e) {
                if (debugMode) {
                    System.out.printf("Error reading CAN data for API %d: %s%n", 
                        listenerPair.getAPIID(), e.getMessage());
                }
            }
        }
    }

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

    protected long pollCanDeviceLong(int apiID) {
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

    protected int pollCanDevice32BitSigned(int apiID, int offset) {
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
    protected int readCanPacketAs32BitSigned(byte[] receivedData, int offset) {
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
    protected byte pollCanDeviceByte(int apiID) {
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
    // Command Sending Methods - FIXED
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
            int canId = getRxIdentifier(apiID);
            canDevice.writePacket(commandData, canId);
            if (debugMode) {
                System.out.printf("Device %d: Sent %s command (CAN_ID=0x%X, API_ID=%d)%n", 
                                deviceID, commandName, canId, apiID);
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
        byte[] commandData = new byte[8];
    
        // Convert to double (8 bytes)
        byte[] valueBytes = doubleToByteArray(value);
        
        // Pack the value into the command data
        System.arraycopy(valueBytes, 0, commandData, 0, 8);

        boolean success = attemptCanOperation(() -> {
            int canId = getTxIdentifier(apiID);
            canDevice.writePacket(commandData, canId);
            if (debugMode) {
                System.out.printf("Device %d: Sent %s command (CAN_ID=0x%X, API_ID=%d, Value=%.6f)%n", 
                                deviceID, commandName, canId, apiID, value);
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
    
        // Convert to float (4 bytes each) to match C code expectation
        byte[] valueBytes = floatToByteArray((float)value);
        byte[] timeoutBytes = floatToByteArray((float)timeout);
        
        // Pack both values into the command data
        System.arraycopy(valueBytes, 0, commandData, 0, 4);
        System.arraycopy(timeoutBytes, 0, commandData, 4, 4);

        boolean success = attemptCanOperation(() -> {
            int canId = getTxIdentifier(apiID);
            canDevice.writePacket(commandData, canId);
            if (debugMode) {
                System.out.printf("Device %d: Sent %s command (CAN_ID=0x%X, API_ID=%d, Value=%.6f, Timeout=%.3f)%n", 
                                deviceID, commandName, canId, apiID, value, timeout);
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
     * Note: The C code doesn't appear to send acknowledgments, so this may always timeout
     * 
     * @param apiID The API ID to listen for acknowledgment
     * @param timeoutMs Maximum time to wait for acknowledgment in milliseconds
     * @return True if acknowledgment received, false if timeout
     */
    protected boolean waitForCommandAck(int apiID, long timeoutMs) {
        long startTime = System.currentTimeMillis();
        
        while (System.currentTimeMillis() - startTime < timeoutMs) {
            int ackCanId = getTxIdentifier(apiID + 100); // Ack API is command API + 100
            if (canDevice.readPacketLatest(ackCanId, canData)) {
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
            int canId = getTxIdentifier(apiID);
            canDevice.writePacket(data, canId);
            if (debugMode) {
                System.out.printf("Device %d: Sent CAN Packet: CAN_ID=0x%X, API_ID=%d, Data=%s%n",
                        deviceID, canId, apiID, bytesToHex(data));
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

        int queryCanId = getTxIdentifier(QUERY_API_ID);

        do {
            packetReceived = canDevice.readPacketNew(queryCanId, canData);
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
        int queryCanId = getTxIdentifier(QUERY_API_ID);
        boolean packetReceived = canDevice.readPacketNew(queryCanId, canData);

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
            int canId = getRxIdentifier(apiID);
            if (canDevice.readPacketLatest(canId, canData)) {
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
    protected long readCanPacketAsLong(byte[] receivedData) {
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
    
    // Helper class to represent a pair of CoreDeviceListener and Integer
    private static class Pair<T, U> {
        private final T first;
        private final U second;

        public Pair(T first, U second) {
            this.first = first;
            this.second = second;
        }

        public T getListener() {
            return first;
        }

        public U getAPIID() {
            return second;
        }
    }
}