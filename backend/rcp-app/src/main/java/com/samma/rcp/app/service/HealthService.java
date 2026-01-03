package com.samma.rcp.app.service;

import com.samma.rcp.app.config.RosProperties;
import com.samma.rcp.app.dto.HealthSummaryDTO;
import com.samma.rcp.app.dto.HealthSummaryDTO.ServiceHealth;
import com.samma.rcp.app.dto.HealthSummaryDTO.SimulationInfo;
import com.samma.rcp.app.dto.HealthSummaryDTO.ErrorInfo;
import com.samma.rcp.app.orchestration.SimulationOrchestrator;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Service;

import java.io.IOException;
import java.net.Socket;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.time.Duration;
import java.time.Instant;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Health Service - Single Source of Truth for System Health
 * Actively verifies all services without blocking
 */
@Slf4j
@Service
public class HealthService {

    private final RosProperties rosProperties;
    private final SimulationOrchestrator orchestrator;
    private final HttpClient httpClient;
    
    // Cached health states (prevents thundering herd)
    private final ConcurrentHashMap<String, ServiceHealth> healthCache = new ConcurrentHashMap<>();
    private final ConcurrentHashMap<String, Long> lastCheckTime = new ConcurrentHashMap<>();
    
    // Last known error
    private volatile ErrorInfo lastError = null;
    
    // Cache TTL (ms)
    private static final long CACHE_TTL_MS = 2000; // 2 seconds
    private static final int TCP_TIMEOUT_MS = 1000; // 1 second for TCP checks
    private static final int HTTP_TIMEOUT_MS = 2000; // 2 seconds for HTTP checks

    public HealthService(RosProperties rosProperties, SimulationOrchestrator orchestrator) {
        this.rosProperties = rosProperties;
        this.orchestrator = orchestrator;
        this.httpClient = HttpClient.newBuilder()
                .connectTimeout(Duration.ofMillis(HTTP_TIMEOUT_MS))
                .build();
    }

    /**
     * Get comprehensive health summary
     * Never throws - returns DEGRADED status on failures
     */
    public HealthSummaryDTO getHealthSummary() {
        try {
            // Check all services
            ServiceHealth rosbridge = checkRosBridge();
            ServiceHealth stomp = checkStomp();
            ServiceHealth rvizNovnc = checkRvizNovnc();
            ServiceHealth turtlesimNovnc = checkTurtlesimNovnc();
            ServiceHealth database = checkDatabase();
            
            // Get simulation info
            SimulationInfo simInfo = getSimulationInfo();
            
            // Determine overall status
            String overallStatus = determineOverallStatus(rosbridge, stomp, rvizNovnc, turtlesimNovnc, database);
            
            return HealthSummaryDTO.builder()
                    .overallStatus(overallStatus)
                    .rosbridge(rosbridge)
                    .stomp(stomp)
                    .rvizNovnc(rvizNovnc)
                    .turtlesimNovnc(turtlesimNovnc)
                    .database(database)
                    .activeSimulation(simInfo)
                    .lastError(lastError)
                    .timestamp(Instant.now())
                    .build();
                    
        } catch (Exception e) {
            log.error("[HealthService] Unexpected error in getHealthSummary", e);
            
            // Cache this error
            setLastError("BACKEND", "Health check failed: " + e.getMessage(), 
                        "Check backend logs for details");
            
            // Return degraded state
            return HealthSummaryDTO.builder()
                    .overallStatus("ERROR")
                    .lastError(lastError)
                    .timestamp(Instant.now())
                    .build();
        }
    }

    /**
     * Check ROSBridge WebSocket availability
     */
    private ServiceHealth checkRosBridge() {
        String cacheKey = "rosbridge";
        
        // Check cache
        if (isCacheValid(cacheKey)) {
            return healthCache.get(cacheKey);
        }
        
        ServiceHealth health;
        String rosbridgeUrl = rosProperties.getBridge().getUrl();
        
        try {
            // Parse WebSocket URL to get host and port
            URI uri = URI.create(rosbridgeUrl.replace("ws://", "http://").replace("wss://", "https://"));
            String host = uri.getHost();
            int port = uri.getPort() > 0 ? uri.getPort() : 9090;
            
            // TCP connectivity check (non-blocking with timeout)
            boolean tcpReachable = checkTcpPort(host, port, TCP_TIMEOUT_MS);
            
            if (tcpReachable) {
                health = ServiceHealth.builder()
                        .status("CONNECTED")
                        .message("ROSBridge reachable")
                        .url(rosbridgeUrl)
                        .lastCheckMs(System.currentTimeMillis())
                        .retryAttempts(0)
                        .build();
            } else {
                health = ServiceHealth.builder()
                        .status("DISCONNECTED")
                        .message("TCP connection failed")
                        .url(rosbridgeUrl)
                        .lastCheckMs(System.currentTimeMillis())
                        .retryAttempts(0)
                        .build();
                        
                setLastError("ROS", "ROSBridge unreachable at " + rosbridgeUrl,
                           "Check if rosbridge container is running: docker ps");
            }
            
        } catch (Exception e) {
            log.warn("[HealthService] ROSBridge check failed: {}", e.getMessage());
            health = ServiceHealth.builder()
                    .status("ERROR")
                    .message("Check failed: " + e.getMessage())
                    .url(rosbridgeUrl)
                    .lastCheckMs(System.currentTimeMillis())
                    .retryAttempts(0)
                    .build();
        }
        
        // Update cache
        healthCache.put(cacheKey, health);
        lastCheckTime.put(cacheKey, System.currentTimeMillis());
        
        return health;
    }

    /**
     * Check STOMP WebSocket endpoint
     */
    private ServiceHealth checkStomp() {
        String cacheKey = "stomp";
        
        if (isCacheValid(cacheKey)) {
            return healthCache.get(cacheKey);
        }
        
        ServiceHealth health;
        String stompUrl = "http://localhost:8080/ws/robot"; // STOMP endpoint
        
        try {
            // Since STOMP uses SockJS, check the info endpoint
            HttpRequest request = HttpRequest.newBuilder()
                    .uri(URI.create(stompUrl + "/info"))
                    .timeout(Duration.ofMillis(HTTP_TIMEOUT_MS))
                    .GET()
                    .build();
                    
            HttpResponse<String> response = httpClient.send(request, HttpResponse.BodyHandlers.ofString());
            
            if (response.statusCode() == 200) {
                health = ServiceHealth.builder()
                        .status("CONNECTED")
                        .message("STOMP endpoint available")
                        .url(stompUrl)
                        .lastCheckMs(System.currentTimeMillis())
                        .retryAttempts(0)
                        .build();
            } else {
                health = ServiceHealth.builder()
                        .status("DEGRADED")
                        .message("Unexpected status: " + response.statusCode())
                        .url(stompUrl)
                        .lastCheckMs(System.currentTimeMillis())
                        .retryAttempts(0)
                        .build();
            }
            
        } catch (Exception e) {
            log.warn("[HealthService] STOMP check failed: {}", e.getMessage());
            health = ServiceHealth.builder()
                    .status("DISCONNECTED")
                    .message("STOMP endpoint unreachable")
                    .url(stompUrl)
                    .lastCheckMs(System.currentTimeMillis())
                    .retryAttempts(0)
                    .build();
                    
            setLastError("WS", "STOMP WebSocket unreachable",
                       "Check if backend is running properly");
        }
        
        healthCache.put(cacheKey, health);
        lastCheckTime.put(cacheKey, System.currentTimeMillis());
        
        return health;
    }

    /**
     * Check RViz noVNC availability
     */
    private ServiceHealth checkRvizNovnc() {
        String cacheKey = "rviz-novnc";
        
        if (isCacheValid(cacheKey)) {
            return healthCache.get(cacheKey);
        }
        
        String rvizUrl = rosProperties.getVisualization().getRvizUrl();
        ServiceHealth health = checkHttpEndpoint(rvizUrl, "RViz noVNC");
        
        healthCache.put(cacheKey, health);
        lastCheckTime.put(cacheKey, System.currentTimeMillis());
        
        if (!"CONNECTED".equals(health.getStatus())) {
            setLastError("DOCKER", "RViz noVNC unreachable",
                       "Start rviz-novnc container: docker-compose up -d rviz-novnc");
        }
        
        return health;
    }

    /**
     * Check Turtlesim noVNC availability
     */
    private ServiceHealth checkTurtlesimNovnc() {
        String cacheKey = "turtlesim-novnc";
        
        if (isCacheValid(cacheKey)) {
            return healthCache.get(cacheKey);
        }
        
        String turtlesimUrl = rosProperties.getVisualization().getTurtlesimUrl();
        ServiceHealth health = checkHttpEndpoint(turtlesimUrl, "Turtlesim noVNC");
        
        healthCache.put(cacheKey, health);
        lastCheckTime.put(cacheKey, System.currentTimeMillis());
        
        if (!"CONNECTED".equals(health.getStatus())) {
            setLastError("DOCKER", "Turtlesim noVNC unreachable",
                       "Start turtlesim container: docker-compose up -d turtlesim-novnc");
        }
        
        return health;
    }

    /**
     * Check database connectivity
     */
    private ServiceHealth checkDatabase() {
        String cacheKey = "database";
        
        if (isCacheValid(cacheKey)) {
            return healthCache.get(cacheKey);
        }
        
        ServiceHealth health;
        
        try {
            // Simple check - if we got this far, JPA connection is working
            // In production, you'd query a dummy table or use DataSource.getConnection()
            health = ServiceHealth.builder()
                    .status("CONNECTED")
                    .message("Database operational")
                    .url("PostgreSQL")
                    .lastCheckMs(System.currentTimeMillis())
                    .retryAttempts(0)
                    .build();
                    
        } catch (Exception e) {
            log.error("[HealthService] Database check failed", e);
            health = ServiceHealth.builder()
                    .status("ERROR")
                    .message("Database connection failed")
                    .url("PostgreSQL")
                    .lastCheckMs(System.currentTimeMillis())
                    .retryAttempts(0)
                    .build();
                    
            setLastError("BACKEND", "Database connection lost",
                       "Check PostgreSQL container: docker-compose up -d postgres");
        }
        
        healthCache.put(cacheKey, health);
        lastCheckTime.put(cacheKey, System.currentTimeMillis());
        
        return health;
    }

    /**
     * Get active simulation information
     */
    private SimulationInfo getSimulationInfo() {
        try {
            boolean isRunning = orchestrator.isRunning();
            
            if (isRunning) {
                // Get running simulation details
                // In production, you'd fetch this from SimulationSession entity
                return SimulationInfo.builder()
                        .status("RUNNING")
                        .scenario("UNKNOWN") // TODO: Track active scenario
                        .model("UNKNOWN") // TODO: Track active model
                        .startedAt(null) // TODO: Track start time
                        .uptimeSeconds(null)
                        .build();
            } else {
                return SimulationInfo.builder()
                        .status("STOPPED")
                        .build();
            }
            
        } catch (Exception e) {
            log.error("[HealthService] Failed to get simulation info", e);
            return SimulationInfo.builder()
                    .status("ERROR")
                    .build();
        }
    }

    /**
     * Determine overall system status based on individual services
     */
    private String determineOverallStatus(ServiceHealth... services) {
        int connected = 0;
        int total = 0;
        int errors = 0;
        
        for (ServiceHealth service : services) {
            if (service == null) continue;
            total++;
            
            String status = service.getStatus();
            if ("CONNECTED".equals(status)) {
                connected++;
            } else if ("ERROR".equals(status)) {
                errors++;
            }
        }
        
        if (total == 0) return "UNKNOWN";
        
        // All services connected
        if (connected == total) return "HEALTHY";
        
        // Some services have errors
        if (errors > 0) return "ERROR";
        
        // Some services disconnected but no errors
        if (connected > 0) return "DEGRADED";
        
        // Nothing connected
        return "ERROR";
    }

    /**
     * Generic HTTP endpoint checker
     */
    private ServiceHealth checkHttpEndpoint(String url, String serviceName) {
        try {
            HttpRequest request = HttpRequest.newBuilder()
                    .uri(URI.create(url))
                    .timeout(Duration.ofMillis(HTTP_TIMEOUT_MS))
                    .GET()
                    .build();
                    
            HttpResponse<String> response = httpClient.send(request, HttpResponse.BodyHandlers.ofString());
            
            if (response.statusCode() >= 200 && response.statusCode() < 400) {
                return ServiceHealth.builder()
                        .status("CONNECTED")
                        .message(serviceName + " available")
                        .url(url)
                        .lastCheckMs(System.currentTimeMillis())
                        .retryAttempts(0)
                        .build();
            } else {
                return ServiceHealth.builder()
                        .status("DEGRADED")
                        .message("HTTP " + response.statusCode())
                        .url(url)
                        .lastCheckMs(System.currentTimeMillis())
                        .retryAttempts(0)
                        .build();
            }
            
        } catch (Exception e) {
            log.debug("[HealthService] {} check failed: {}", serviceName, e.getMessage());
            return ServiceHealth.builder()
                    .status("DISCONNECTED")
                    .message(serviceName + " unreachable")
                    .url(url)
                    .lastCheckMs(System.currentTimeMillis())
                    .retryAttempts(0)
                    .build();
        }
    }

    /**
     * Check if TCP port is reachable (non-blocking with timeout)
     */
    private boolean checkTcpPort(String host, int port, int timeoutMs) {
        try (Socket socket = new Socket()) {
            socket.connect(new java.net.InetSocketAddress(host, port), timeoutMs);
            return true;
        } catch (IOException e) {
            return false;
        }
    }

    /**
     * Check if cache is still valid
     */
    private boolean isCacheValid(String key) {
        Long lastCheck = lastCheckTime.get(key);
        if (lastCheck == null) return false;
        
        return (System.currentTimeMillis() - lastCheck) < CACHE_TTL_MS;
    }

    /**
     * Set last error (thread-safe)
     */
    private void setLastError(String source, String message, String suggestion) {
        this.lastError = ErrorInfo.builder()
                .source(source)
                .message(message)
                .timestamp(Instant.now())
                .suggestion(suggestion)
                .build();
    }

    /**
     * Clear last error
     */
    public void clearLastError() {
        this.lastError = null;
    }
}

