package com.samma.rcp.app.controller;

import com.samma.rcp.app.dto.HealthSummaryDTO;
import com.samma.rcp.app.service.HealthService;
import com.samma.rcp.base.controller.BaseController;
import com.samma.rcp.base.dto.ResponseDTO;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

/**
 * Health Controller - Single Source of Truth Endpoint
 * 
 * Provides unified health summary for frontend consumption
 * NEVER returns HTTP 500 - always returns valid HealthSummaryDTO
 */
@Slf4j
@RestController
@RequestMapping("/api/health")
@RequiredArgsConstructor
public class HealthController extends BaseController {

    private final HealthService healthService;

    /**
     * Get comprehensive system health summary
     * 
     * This is the ONLY endpoint Dashboard should use for health status
     * 
     * @return Always returns 200 OK with HealthSummaryDTO
     *         On failures, status field will be DEGRADED or ERROR with reasons
     */
    @GetMapping("/summary")
    public ResponseEntity<ResponseDTO<HealthSummaryDTO>> getHealthSummary() {
        try {
            HealthSummaryDTO summary = healthService.getHealthSummary();
            
            // Always return 200 OK - status is in the DTO
            return success(summary);
            
        } catch (Exception e) {
            // Even if health check itself fails, return gracefully
            log.error("[HealthController] Unexpected error in /summary", e);
            
            HealthSummaryDTO fallback = HealthSummaryDTO.builder()
                    .overallStatus("ERROR")
                    .lastError(HealthSummaryDTO.ErrorInfo.builder()
                            .source("BACKEND")
                            .message("Health check system failure")
                            .suggestion("Check backend logs immediately")
                            .timestamp(java.time.Instant.now())
                            .build())
                    .timestamp(java.time.Instant.now())
                    .build();
                    
            return success(fallback);
        }
    }

    /**
     * Clear last error from health service
     * 
     * Useful when user acknowledges an error and wants to clear it
     */
    @PostMapping("/clear-error")
    public ResponseEntity<ResponseDTO<String>> clearLastError() {
        try {
            healthService.clearLastError();
            return success("Last error cleared");
        } catch (Exception e) {
            log.error("[HealthController] Failed to clear error", e);
            return success("Error clearing failed"); // Still return 200
        }
    }

    /**
     * Lightweight ping endpoint for basic backend reachability
     * 
     * Does not perform deep health checks - just confirms backend is alive
     */
    @GetMapping("/ping")
    public ResponseEntity<ResponseDTO<String>> ping() {
        return success("pong");
    }
}

