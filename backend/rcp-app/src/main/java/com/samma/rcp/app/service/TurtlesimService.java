package com.samma.rcp.app.service;

import com.samma.rcp.app.dto.TwistDTO;
import com.samma.rcp.app.orchestration.RosBridgeClient;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.HashMap;
import java.util.Map;
import java.util.UUID;

@Slf4j
@Service
@RequiredArgsConstructor
public class TurtlesimService {

    private final RosBridgeClient rosBridgeClient;

    @Value("${ros.turtlesim.topic:/turtle1/cmd_vel}")
    private String turtlesimTopic;

    public void sendCmdVel(TwistDTO dto) {
        // #region agent log
        try {
            boolean isConnected = rosBridgeClient.isConnected();
            String logLine = String.format("{\"id\":\"%s\",\"timestamp\":%d,\"location\":\"TurtlesimService.sendCmdVel:26\",\"message\":\"Entry\",\"data\":{\"linear\":%f,\"angular\":%f,\"topic\":\"%s\",\"rosBridgeConnected\":%s},\"sessionId\":\"debug-session\",\"runId\":\"run1\",\"hypothesisId\":\"B\"}\n",
                UUID.randomUUID(), System.currentTimeMillis(), dto.getLinear(), dto.getAngular(), turtlesimTopic, isConnected);
            String logDir = System.getenv().getOrDefault("DEBUG_LOG_DIR", System.getProperty("user.dir", "/app"));
            Path logPath = Paths.get(logDir, ".cursor", "debug.log");
            Files.write(logPath, logLine.getBytes(), StandardOpenOption.CREATE, StandardOpenOption.APPEND);
        } catch (Exception ignored) {}
        // #endregion
        // Override topic to use turtlesim topic
        TwistDTO turtlesimDto = TwistDTO.builder()
                .linear(dto.getLinear())
                .angular(dto.getAngular())
                .topic(turtlesimTopic)
                .build();
        
        log.debug("Publishing to {}: linear={}, angular={}", turtlesimTopic, dto.getLinear(), dto.getAngular());
        try {
            rosBridgeClient.publishTwist(turtlesimDto);
            // #region agent log
            try {
                String logLine = String.format("{\"id\":\"%s\",\"timestamp\":%d,\"location\":\"TurtlesimService.sendCmdVel:42\",\"message\":\"publishTwist success\",\"data\":{},\"sessionId\":\"debug-session\",\"runId\":\"run1\",\"hypothesisId\":\"B\"}\n",
                    UUID.randomUUID(), System.currentTimeMillis());
                String logDir = System.getenv().getOrDefault("DEBUG_LOG_DIR", System.getProperty("user.dir", "/app"));
            Path logPath = Paths.get(logDir, ".cursor", "debug.log");
                Files.write(logPath, logLine.getBytes(), StandardOpenOption.CREATE, StandardOpenOption.APPEND);
            } catch (Exception ignored) {}
            // #endregion
        } catch (Exception e) {
            // #region agent log
            try {
                String logLine = String.format("{\"id\":\"%s\",\"timestamp\":%d,\"location\":\"TurtlesimService.sendCmdVel:50\",\"message\":\"publishTwist exception\",\"data\":{\"exception\":\"%s\",\"message\":\"%s\"},\"sessionId\":\"debug-session\",\"runId\":\"run1\",\"hypothesisId\":\"B\"}\n",
                    UUID.randomUUID(), System.currentTimeMillis(), e.getClass().getName(), e.getMessage().replace("\"", "\\\""));
                String logDir = System.getenv().getOrDefault("DEBUG_LOG_DIR", System.getProperty("user.dir", "/app"));
            Path logPath = Paths.get(logDir, ".cursor", "debug.log");
                Files.write(logPath, logLine.getBytes(), StandardOpenOption.CREATE, StandardOpenOption.APPEND);
            } catch (Exception ignored) {}
            // #endregion
            throw e;
        }
    }

    public Map<String, Object> getStatus() {
        Map<String, Object> status = new HashMap<>();
        boolean connected = rosBridgeClient.isConnected();
        status.put("running", connected);
        status.put("topic", turtlesimTopic);
        status.put("connected", connected);
        return status;
    }
}

