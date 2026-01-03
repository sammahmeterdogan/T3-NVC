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
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.time.Instant;
import java.util.Map;
import java.util.UUID;

@Slf4j
@Component
@RequiredArgsConstructor
public class RosBridgeClient {

    private final ObjectMapper om;

    @Getter
    private volatile boolean connected = false;
    private WebSocketClient client;
    private String currentUrl;

    public synchronized void connect(String wsUrl) {
        if (connected && wsUrl.equals(currentUrl)) return;
        try {
            currentUrl = wsUrl;
            if (client != null && client.isOpen()) client.close();

            client = new WebSocketClient(new URI(wsUrl)) {
                @Override public void onOpen(ServerHandshake h) { connected = true; log.info("rosbridge connected {}", wsUrl); }
                @Override public void onMessage(String message) { /* no-op */ }
                @Override public void onClose(int code, String reason, boolean remote) { connected = false; log.warn("rosbridge closed: {}", reason); }
                @Override public void onError(Exception ex) { connected = false; log.error("rosbridge error", ex); }
            };
            client.connectBlocking();
        } catch (Exception e) {
            connected = false;
            throw new RuntimeException("rosbridge connect failed: " + wsUrl, e);
        }
    }

    public synchronized void disconnect() {
        try { if (client != null) client.closeBlocking(); } catch (Exception ignored) {}
        connected = false;
    }

    private void send(Map<String, Object> payload) {
        // #region agent log
        try {
            boolean clientNull = (client == null);
            boolean clientOpen = (client != null && client.isOpen());
            String logLine = String.format("{\"id\":\"%s\",\"timestamp\":%d,\"location\":\"RosBridgeClient.send:57\",\"message\":\"Entry\",\"data\":{\"clientNull\":%s,\"clientOpen\":%s,\"connected\":%s},\"sessionId\":\"debug-session\",\"runId\":\"run1\",\"hypothesisId\":\"A\"}\n",
                UUID.randomUUID(), System.currentTimeMillis(), clientNull, clientOpen, connected);
            Path logPath = Paths.get(System.getProperty("user.dir", "/app"), ".cursor", "debug.log");
            Files.write(logPath, logLine.getBytes(), StandardOpenOption.CREATE, StandardOpenOption.APPEND);
        } catch (Exception ignored) {}
        // #endregion
        try {
            if (client == null) {
                // #region agent log
                try {
                    String logLine = String.format("{\"id\":\"%s\",\"timestamp\":%d,\"location\":\"RosBridgeClient.send:66\",\"message\":\"Client is null\",\"data\":{},\"sessionId\":\"debug-session\",\"runId\":\"run1\",\"hypothesisId\":\"A\"}\n",
                        UUID.randomUUID(), System.currentTimeMillis());
                    Path logPath = Paths.get(System.getProperty("user.dir", "/app"), ".cursor", "debug.log");
                    Files.write(logPath, logLine.getBytes(), StandardOpenOption.CREATE, StandardOpenOption.APPEND);
                } catch (Exception ignored) {}
                // #endregion
                throw new RuntimeException("rosbridge client is null - not connected");
            }
            String json = om.writeValueAsString(payload);
            client.send(json);
            // #region agent log
            try {
                String logLine = String.format("{\"id\":\"%s\",\"timestamp\":%d,\"location\":\"RosBridgeClient.send:76\",\"message\":\"Send success\",\"data\":{},\"sessionId\":\"debug-session\",\"runId\":\"run1\",\"hypothesisId\":\"A\"}\n",
                    UUID.randomUUID(), System.currentTimeMillis());
                String logDir = System.getenv().getOrDefault("DEBUG_LOG_DIR", System.getProperty("user.dir", "/app"));
            Path logPath = Paths.get(logDir, ".cursor", "debug.log");
                Files.write(logPath, logLine.getBytes(), StandardOpenOption.CREATE, StandardOpenOption.APPEND);
            } catch (Exception ignored) {}
            // #endregion
        } catch (Exception e) {
            // #region agent log
            try {
                String logLine = String.format("{\"id\":\"%s\",\"timestamp\":%d,\"location\":\"RosBridgeClient.send:84\",\"message\":\"Send exception\",\"data\":{\"exception\":\"%s\",\"message\":\"%s\"},\"sessionId\":\"debug-session\",\"runId\":\"run1\",\"hypothesisId\":\"A\"}\n",
                    UUID.randomUUID(), System.currentTimeMillis(), e.getClass().getName(), e.getMessage().replace("\"", "\\\""));
                String logDir = System.getenv().getOrDefault("DEBUG_LOG_DIR", System.getProperty("user.dir", "/app"));
            Path logPath = Paths.get(logDir, ".cursor", "debug.log");
                Files.write(logPath, logLine.getBytes(), StandardOpenOption.CREATE, StandardOpenOption.APPEND);
            } catch (Exception ignored) {}
            // #endregion
            throw new RuntimeException("rosbridge send failed", e);
        }
    }

    private void advertise(String topic, String type) {
        send(Map.of("op", "advertise", "id", "adv-"+topic, "topic", topic, "type", type));
    }

    private void publish(String topic, Object msg) {
        send(Map.of("op", "publish", "id", "pub-"+UUID.randomUUID(), "topic", topic, "msg", msg));
    }

    public void publishTwist(TwistDTO dto) {
        // #region agent log
        try {
            String logLine = String.format("{\"id\":\"%s\",\"timestamp\":%d,\"location\":\"RosBridgeClient.publishTwist:70\",\"message\":\"Entry\",\"data\":{\"topic\":\"%s\",\"linear\":%f,\"angular\":%f,\"connected\":%s},\"sessionId\":\"debug-session\",\"runId\":\"run1\",\"hypothesisId\":\"C\"}\n",
                UUID.randomUUID(), System.currentTimeMillis(), dto.getTopic() == null ? "/cmd_vel" : dto.getTopic(), dto.getLinear(), dto.getAngular(), connected);
            String logDir = System.getenv().getOrDefault("DEBUG_LOG_DIR", System.getProperty("user.dir", "/app"));
            Path logPath = Paths.get(logDir, ".cursor", "debug.log");
            Files.write(logPath, logLine.getBytes(), StandardOpenOption.CREATE, StandardOpenOption.APPEND);
        } catch (Exception ignored) {}
        // #endregion
        String topic = dto.getTopic() == null ? "/cmd_vel" : dto.getTopic();
        try {
            advertise(topic, "geometry_msgs/msg/Twist");
            // #region agent log
            try {
                String logLine = String.format("{\"id\":\"%s\",\"timestamp\":%d,\"location\":\"RosBridgeClient.publishTwist:81\",\"message\":\"Advertise success\",\"data\":{},\"sessionId\":\"debug-session\",\"runId\":\"run1\",\"hypothesisId\":\"C\"}\n",
                    UUID.randomUUID(), System.currentTimeMillis());
                String logDir = System.getenv().getOrDefault("DEBUG_LOG_DIR", System.getProperty("user.dir", "/app"));
            Path logPath = Paths.get(logDir, ".cursor", "debug.log");
                Files.write(logPath, logLine.getBytes(), StandardOpenOption.CREATE, StandardOpenOption.APPEND);
            } catch (Exception ignored) {}
            // #endregion
        } catch (Exception e) {
            // #region agent log
            try {
                String logLine = String.format("{\"id\":\"%s\",\"timestamp\":%d,\"location\":\"RosBridgeClient.publishTwist:89\",\"message\":\"Advertise exception\",\"data\":{\"exception\":\"%s\"},\"sessionId\":\"debug-session\",\"runId\":\"run1\",\"hypothesisId\":\"C\"}\n",
                    UUID.randomUUID(), System.currentTimeMillis(), e.getClass().getName());
                String logDir = System.getenv().getOrDefault("DEBUG_LOG_DIR", System.getProperty("user.dir", "/app"));
            Path logPath = Paths.get(logDir, ".cursor", "debug.log");
                Files.write(logPath, logLine.getBytes(), StandardOpenOption.CREATE, StandardOpenOption.APPEND);
            } catch (Exception ignored) {}
            // #endregion
            throw e;
        }
        Map<String, Object> msg = Map.of(
                "linear", Map.of("x", dto.getLinear(), "y", 0.0, "z", 0.0),
                "angular", Map.of("x", 0.0, "y", 0.0, "z", dto.getAngular())
        );
        try {
            publish(topic, msg);
            // #region agent log
            try {
                String logLine = String.format("{\"id\":\"%s\",\"timestamp\":%d,\"location\":\"RosBridgeClient.publishTwist:103\",\"message\":\"Publish success\",\"data\":{},\"sessionId\":\"debug-session\",\"runId\":\"run1\",\"hypothesisId\":\"C\"}\n",
                    UUID.randomUUID(), System.currentTimeMillis());
                String logDir = System.getenv().getOrDefault("DEBUG_LOG_DIR", System.getProperty("user.dir", "/app"));
            Path logPath = Paths.get(logDir, ".cursor", "debug.log");
                Files.write(logPath, logLine.getBytes(), StandardOpenOption.CREATE, StandardOpenOption.APPEND);
            } catch (Exception ignored) {}
            // #endregion
        } catch (Exception e) {
            // #region agent log
            try {
                String logLine = String.format("{\"id\":\"%s\",\"timestamp\":%d,\"location\":\"RosBridgeClient.publishTwist:111\",\"message\":\"Publish exception\",\"data\":{\"exception\":\"%s\"},\"sessionId\":\"debug-session\",\"runId\":\"run1\",\"hypothesisId\":\"C\"}\n",
                    UUID.randomUUID(), System.currentTimeMillis(), e.getClass().getName());
                String logDir = System.getenv().getOrDefault("DEBUG_LOG_DIR", System.getProperty("user.dir", "/app"));
            Path logPath = Paths.get(logDir, ".cursor", "debug.log");
                Files.write(logPath, logLine.getBytes(), StandardOpenOption.CREATE, StandardOpenOption.APPEND);
            } catch (Exception ignored) {}
            // #endregion
            throw e;
        }
    }

    public void sendGoal(GoalPoseDTO goal) {
        String frame = goal.getFrameId() == null ? "map" : goal.getFrameId();
        advertise("/goal_pose", "geometry_msgs/msg/PoseStamped");
        Map<String, Object> msg = Map.of(
                "header", Map.of("stamp", Map.of("sec", Instant.now().getEpochSecond(), "nanosec", 0), "frame_id", frame),
                "pose", Map.of(
                        "position", Map.of("x", goal.getX(), "y", goal.getY(), "z", 0.0),
                        "orientation", yawToQuat(goal.getTheta())
                )
        );
        publish("/goal_pose", msg);
    }

    private Map<String, Object> yawToQuat(double yaw) {
        double cy = Math.cos(yaw * 0.5), sy = Math.sin(yaw * 0.5);
        return Map.of("x", 0.0, "y", 0.0, "z", sy, "w", cy);
    }
}
