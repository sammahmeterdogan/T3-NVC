package com.samma.rcp.app.config;

import com.samma.rcp.app.orchestration.RosBridgeClient;
import jakarta.annotation.PostConstruct;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.boot.autoconfigure.condition.ConditionalOnProperty;
import org.springframework.stereotype.Component;

@Slf4j
@Component
@RequiredArgsConstructor
@ConditionalOnProperty(name = "ros.bridge.url")
public class RosBridgeConfig {

    private final RosBridgeClient rosBridgeClient;
    private final RosProperties rosProperties;

    @PostConstruct
    public void init() {
        String wsUrl = rosProperties.getBridge().getUrl();
        if (wsUrl != null && !wsUrl.isEmpty()) {
            try {
                log.info("Connecting to rosbridge at: {}", wsUrl);
                rosBridgeClient.connect(wsUrl);
                log.info("RosBridgeClient initialized successfully");
            } catch (Exception e) {
                log.error("Failed to connect to rosbridge at {}: {}", wsUrl, e.getMessage(), e);
                // Don't throw - allow app to start even if rosbridge is unavailable
            }
        } else {
            log.warn("ros.bridge.url is not configured, RosBridgeClient will not connect");
        }
    }
}

