package com.samma.rcp.app.dto;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.time.Instant;

/**
 * Unified health summary for Dashboard
 * Single source of truth for system-wide health status
 */
@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class HealthSummaryDTO {
    
    // Overall system status
    private String overallStatus; // HEALTHY | DEGRADED | ERROR | STARTING
    
    // Individual service statuses
    private ServiceHealth rosbridge;
    private ServiceHealth stomp;
    private ServiceHealth rvizNovnc;
    private ServiceHealth turtlesimNovnc;
    private ServiceHealth database;
    
    // Active simulation info
    private SimulationInfo activeSimulation;
    
    // Last error (if any)
    private ErrorInfo lastError;
    
    // Timestamp
    private Instant timestamp;
    
    @Data
    @Builder
    @NoArgsConstructor
    @AllArgsConstructor
    public static class ServiceHealth {
        private String status; // CONNECTED | DISCONNECTED | STARTING | ERROR
        private String message;
        private String url;
        private Long lastCheckMs;
        private Integer retryAttempts;
    }
    
    @Data
    @Builder
    @NoArgsConstructor
    @AllArgsConstructor
    public static class SimulationInfo {
        private String status; // RUNNING | STOPPED | STARTING | ERROR
        private String scenario;
        private String model;
        private Instant startedAt;
        private Long uptimeSeconds;
    }
    
    @Data
    @Builder
    @NoArgsConstructor
    @AllArgsConstructor
    public static class ErrorInfo {
        private String source; // ROS | WS | BACKEND | DOCKER
        private String message;
        private Instant timestamp;
        private String suggestion; // User-friendly fix hint
    }
}

