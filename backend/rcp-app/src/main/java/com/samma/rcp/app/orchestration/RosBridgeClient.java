package com.samma.rcp.app.orchestration;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.samma.rcp.app.dto.GoalPoseDTO;
import com.samma.rcp.app.dto.TwistDTO;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.java_websocket.client.WebSocketClient;
import org.java_websocket.handshake.ServerHandshake;
import org.springframework.stereotype.Component;

import java.net.URI;
import java.time.Instant;
import java.util.Map;
import java.util.UUID;
import java.util.concurrent.TimeUnit;

/**
 * ROSBridge WebSocket client with automatic reconnection support.
 * 
 * Responsibilities:
 * - Maintains WebSocket connection to rosbridge
 * - Auto-reconnects on connection loss before sending commands
 * - Publishes Twist and GoalPose messages to ROS topics
 */
@Slf4j
@Component
@RequiredArgsConstructor
public class RosBridgeClient {

    private final ObjectMapper om;

    @Getter
    private volatile boolean connected = false;
    private WebSocketClient client;
    private String currentUrl;
    
    private static final int MAX_RECONNECT_ATTEMPTS = 3;
    private static final long RECONNECT_DELAY_MS = 500;

    /**
     * Connects to rosbridge WebSocket server.
     * If already connected to the same URL, returns immediately.
     */
    public synchronized void connect(String wsUrl) {
        if (connected && wsUrl.equals(currentUrl)) {
            return;
        }
        
        try {
            currentUrl = wsUrl;
            closeExistingConnection();
            createAndConnectClient(wsUrl);
        } catch (Exception e) {
            connected = false;
            throw new RuntimeException("rosbridge connect failed: " + wsUrl, e);
        }
    }

    /**
     * Disconnects from rosbridge.
     */
    public synchronized void disconnect() {
        try {
            if (client != null) {
                client.closeBlocking();
            }
        } catch (Exception e) {
            log.warn("Error during disconnect", e);
        }
        connected = false;
        client = null;
    }

    /**
     * Publishes a Twist message for robot velocity control.
     * Auto-reconnects if not connected.
     */
    public void publishTwist(TwistDTO dto) {
        ensureConnected();
        
        String topic = dto.getTopic() != null ? dto.getTopic() : "/cmd_vel";
        advertise(topic, "geometry_msgs/msg/Twist");
        
        Map<String, Object> msg = Map.of(
                "linear", Map.of("x", dto.getLinear(), "y", 0.0, "z", 0.0),
                "angular", Map.of("x", 0.0, "y", 0.0, "z", dto.getAngular())
        );
        publish(topic, msg);
    }

    /**
     * Publishes a goal pose for navigation.
     * Auto-reconnects if not connected.
     */
    public void sendGoal(GoalPoseDTO goal) {
        ensureConnected();
        
        String frame = goal.getFrameId() != null ? goal.getFrameId() : "map";
        advertise("/goal_pose", "geometry_msgs/msg/PoseStamped");
        
        Map<String, Object> msg = Map.of(
                "header", Map.of(
                        "stamp", Map.of("sec", Instant.now().getEpochSecond(), "nanosec", 0), 
                        "frame_id", frame
                ),
                "pose", Map.of(
                        "position", Map.of("x", goal.getX(), "y", goal.getY(), "z", 0.0),
                        "orientation", yawToQuaternion(goal.getTheta())
                )
        );
        publish("/goal_pose", msg);
    }

    /**
     * Ensures connection is established, attempting reconnect if needed.
     * Throws RuntimeException if reconnection fails.
     */
    private synchronized void ensureConnected() {
        if (isConnectionActive()) {
            return;
        }
        
        if (currentUrl == null) {
            throw new RuntimeException("ROSBridge URL not configured. Call connect() first.");
        }
        
        log.info("ROSBridge connection lost, attempting reconnect to {}", currentUrl);
        
        for (int attempt = 1; attempt <= MAX_RECONNECT_ATTEMPTS; attempt++) {
            try {
                closeExistingConnection();
                createAndConnectClient(currentUrl);
                
                if (connected) {
                    log.info("ROSBridge reconnected successfully on attempt {}", attempt);
                    return;
                }
            } catch (Exception e) {
                log.warn("Reconnect attempt {} failed: {}", attempt, e.getMessage());
                
                if (attempt < MAX_RECONNECT_ATTEMPTS) {
                    try {
                        TimeUnit.MILLISECONDS.sleep(RECONNECT_DELAY_MS * attempt);
                    } catch (InterruptedException ie) {
                        Thread.currentThread().interrupt();
                        throw new RuntimeException("Reconnection interrupted", ie);
                    }
                }
            }
        }
        
        throw new RuntimeException("Failed to reconnect to ROSBridge after " + MAX_RECONNECT_ATTEMPTS + " attempts");
    }

    private boolean isConnectionActive() {
        return connected && client != null && client.isOpen();
    }

    private void closeExistingConnection() {
        if (client != null) {
            try {
                if (client.isOpen()) {
                    client.close();
                }
            } catch (Exception e) {
                log.debug("Error closing existing connection", e);
            }
            client = null;
        }
    }

    private void createAndConnectClient(String wsUrl) throws Exception {
        client = new WebSocketClient(new URI(wsUrl)) {
            @Override
            public void onOpen(ServerHandshake handshake) {
                connected = true;
                log.info("ROSBridge connected to {}", wsUrl);
            }

            @Override
            public void onMessage(String message) {
                // ROSBridge responses are not currently processed
            }

            @Override
            public void onClose(int code, String reason, boolean remote) {
                connected = false;
                log.warn("ROSBridge connection closed: {} (code: {}, remote: {})", reason, code, remote);
            }

            @Override
            public void onError(Exception ex) {
                connected = false;
                log.error("ROSBridge error", ex);
            }
        };
        
        boolean success = client.connectBlocking(5, TimeUnit.SECONDS);
        if (!success) {
            throw new RuntimeException("Connection timed out");
        }
    }

    private void send(Map<String, Object> payload) {
        if (client == null || !client.isOpen()) {
            throw new RuntimeException("ROSBridge client is not connected");
        }
        
        try {
            String json = om.writeValueAsString(payload);
            client.send(json);
        } catch (Exception e) {
            throw new RuntimeException("Failed to send message to ROSBridge", e);
        }
    }

    private void advertise(String topic, String type) {
        send(Map.of(
                "op", "advertise",
                "id", "adv-" + topic,
                "topic", topic,
                "type", type
        ));
    }

    private void publish(String topic, Object msg) {
        send(Map.of(
                "op", "publish",
                "id", "pub-" + UUID.randomUUID(),
                "topic", topic,
                "msg", msg
        ));
    }

    private Map<String, Object> yawToQuaternion(double yaw) {
        double cy = Math.cos(yaw * 0.5);
        double sy = Math.sin(yaw * 0.5);
        return Map.of("x", 0.0, "y", 0.0, "z", sy, "w", cy);
    }
}
