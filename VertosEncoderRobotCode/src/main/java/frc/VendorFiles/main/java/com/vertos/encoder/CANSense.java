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
    private final double CountsPerRevolution = 2097152.0; // 2 ^ 21
    
    // Fixed-point scale factor (must match C code)
    private static final double FIXED_POINT_SCALE = 65536.0;
    
    // Default API ID for reading multi-turn counts
    private static final int POSITION_API_ID = 0;  // API ID for position data
    private static final int VELOCITY_API_ID = 1; // API ID for velocity data
    private static final int ACCELERATION_API_ID = 2; // API ID for acceleration data
    private static final int QUERY_API_ID = 1;   // API ID for querying devices

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
    public void start() {
        scheduler.scheduleAtFixedRate(() -> {
            encoderThread(); // Call the method to read encoder data
        }, 0, 10, TimeUnit.MILLISECONDS); // Poll every 10ms
    }

    public void stop() {
        scheduler.shutdown(); // Stop the loop
    }

    public void encoderThread() {
        
        // Read absolute rotations (8 bytes)
        if (canDevice.readPacketLatest(POSITION_API_ID, canData)) {
            byte[] receivedData = canData.data;
        
            if (canData.length >= 8) {
                // Extract 64-bit double from 8 bytes (big-endian)
                long absoluteBits = readCanPacket(receivedData);
                // Convert bits to double - perfect precision!
                this.absoluteRotations = Double.longBitsToDouble(absoluteBits);
                // Calculate relative rotations from absolute (0.0 to 1.0)
                this.relativeRotations = absoluteRotations - Math.floor(absoluteRotations);
                // Calculate equivalent multi-turn counts for backward compatibility
                multiTurnCounts = (long)(absoluteRotations * CountsPerRevolution);
        
                if (debugMode) {
                    // System.out.printf(
                    //     "Device %d: absoluteRotations = %.12f, relativeRotations = %.12f%n",
                    //     deviceID, absoluteRotations, relativeRotations
                    // );
                }
            }
        }

        // Read velocity (8 bytes)  
        if (canDevice.readPacketLatest(VELOCITY_API_ID, canData)) {
            byte[] receivedData = canData.data;
            if (canData.length >= 8) {
                // Extract 64-bit velocity double
                long velocityBits = readCanPacket(receivedData);
                // Convert bits to double
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
        if (canDevice.readPacketLatest(ACCELERATION_API_ID, canData)) {
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

    /*
     * 
     *  Read Can Packet data
     * 
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


    public double getAbsRotations() {
        return absoluteRotations;
    }

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

    /**
     * Alternative method for backward compatibility.
     * @deprecated Use getSensorVelocityRPS() instead for clearer units.
     */
    @Deprecated
    public long getSensorVelocity() {
        // Return velocity as scaled long for backward compatibility
        return (long)(velocityRPS * FIXED_POINT_SCALE);
    }

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