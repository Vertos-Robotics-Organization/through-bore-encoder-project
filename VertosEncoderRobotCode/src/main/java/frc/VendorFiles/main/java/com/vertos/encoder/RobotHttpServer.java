package frc.VendorFiles.main.java.com.vertos.encoder;

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import com.sun.net.httpserver.HttpServer;

import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.List;

/**
 * Minimal HTTP server with:
 * 1) "/" (GET) -> "Hello from roboRIO!"
 * 2) "/postData" (POST) -> expects val1=INT&val2=INT
 */
public class RobotHttpServer {
    private HttpServer server;
    private int port;

    // Keep track of the last received integers (optional)
    private volatile int lastVal1 = 0;
    private volatile int lastVal2 = 0;

    public RobotHttpServer(int port) throws IOException {
        this.port = port;
        server = HttpServer.create(new InetSocketAddress(port), 0);

        // Endpoint for simple GET
        server.createContext("/", new HelloHandler());
        // Endpoint for receiving two integers
        server.createContext("/postData", new PostDataHandler());
    }

    public void start() {
        server.start();
        System.out.println("RobotHttpServer started on port " + port);
    }

    public void stop() {
        server.stop(0);
        System.out.println("RobotHttpServer stopped.");
    }

    /**
     * GET request handler at "/"
     */
    private class HelloHandler implements HttpHandler {
        @Override
        public void handle(HttpExchange exchange) throws IOException {
            if (!"GET".equalsIgnoreCase(exchange.getRequestMethod())) {
                exchange.sendResponseHeaders(405, -1); // Method not allowed
                return;
            }

            String response = "Hello from roboRIO!";
            byte[] respBytes = response.getBytes(StandardCharsets.UTF_8);

            exchange.sendResponseHeaders(200, respBytes.length);
            try (OutputStream os = exchange.getResponseBody()) {
                os.write(respBytes);
            }
        }
    }

    /**
     * POST request handler at "/postData"
     * Expects "val1=INT&val2=INT" in the request body.
     */
    private class PostDataHandler implements HttpHandler {
        @Override
        public void handle(HttpExchange exchange) throws IOException {
            if (!"POST".equalsIgnoreCase(exchange.getRequestMethod())) {
                exchange.sendResponseHeaders(405, -1); // Method not allowed
                return;
            }

            // Read request body as a UTF-8 string
            String body = new String(exchange.getRequestBody().readAllBytes(), StandardCharsets.UTF_8);

            // Parse the form data (val1=...&val2=...)
            // We'll split on '&' and then '='
            HashMap<String, String> paramMap = new HashMap<>();
            for (String pair : body.split("&")) {
                String[] kv = pair.split("=");
                if (kv.length == 2) {
                    paramMap.put(kv[0], kv[1]);
                }
            }

            // Extract val1 and val2 as integers
            int val1 = 0;
            int val2 = 0;
            try {
                if (paramMap.containsKey("val1")) {
                    val1 = Integer.parseInt(paramMap.get("val1"));
                }
                if (paramMap.containsKey("val2")) {
                    val2 = Integer.parseInt(paramMap.get("val2"));
                }
            } catch (NumberFormatException e) {
                // If parsing fails, respond with error
                String errorMsg = "Invalid integer(s) provided.";
                byte[] respBytes = errorMsg.getBytes(StandardCharsets.UTF_8);
                exchange.sendResponseHeaders(400, respBytes.length);
                try (OutputStream os = exchange.getResponseBody()) {
                    os.write(respBytes);
                }
                return;
            }

            // Save them (optional)
            lastVal1 = val1;
            lastVal2 = val2;

            System.out.println("Received val1=" + val1 + ", val2=" + val2);

            MT6835 confDevice;
            confDevice = new MT6835(0, false); // Debug mode enabled
            byte[] data = new byte[3]; // Create a byte array with length 3

            // Store the first integer at index 0
            data[0] = (byte) (val1 & 0xFF); // Only store the least significant byte of the first integer

            // Store the second integer across index 1 and 2
            data[1] = (byte) ((val2 >> 8) & 0xFF); // Most significant byte
            data[2] = (byte) (val2 & 0xFF); // Least significant byte
            confDevice.sendPacket(0, data);
            try {
                Thread.sleep(200); // 1000 ms delay
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt(); // Restore the interrupted status
            }
            if (val1 == 44) {
                List<byte[]> receivedPackets = confDevice.queryDevices(); // Call queryDevices

                StringBuilder responseBuilder = new StringBuilder();
                if (!receivedPackets.isEmpty()) {
                    // Format the response with packet data
                    responseBuilder.append("Received Packets: ");
                    for (byte[] packet : receivedPackets) {
                        responseBuilder.append(bytesToHex(packet)).append("; ");
                    }
                } else {
                    responseBuilder.append("No CAN packets received.");
                }

                byte[] respBytes = responseBuilder.toString().getBytes(StandardCharsets.UTF_8);
                exchange.sendResponseHeaders(200, respBytes.length);
                try (OutputStream os = exchange.getResponseBody()) {
                    os.write(respBytes);
                }

            } else {
                // Respond if val1 is not 44
                String response = "RoboRIO got val1=" + val1 + " and val2=" + val2;
                byte[] respBytes = response.getBytes(StandardCharsets.UTF_8);

                exchange.sendResponseHeaders(200, respBytes.length);
                try (OutputStream os = exchange.getResponseBody()) {
                    os.write(respBytes);
                }

            }

        }
    }

    // Optionally provide getters if you need them:
    public int getLastVal1() {
        return lastVal1;
    }

    public int getLastVal2() {
        return lastVal2;
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
}
