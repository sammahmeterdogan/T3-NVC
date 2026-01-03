package com.samma.rcp.app.controller;

import com.samma.rcp.app.dto.TwistDTO;
import com.samma.rcp.app.service.TurtlesimService;
import com.samma.rcp.base.controller.BaseController;
import com.samma.rcp.base.dto.ResponseDTO;
import jakarta.validation.Valid;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.Map;
import java.util.UUID;

@Slf4j
@RestController
@RequestMapping("/api/turtlesim")
@RequiredArgsConstructor
public class TurtlesimController extends BaseController {

    private final TurtlesimService turtlesimService;

    @GetMapping("/status")
    public ResponseEntity<ResponseDTO<Map<String, Object>>> getStatus() {
        Map<String, Object> status = turtlesimService.getStatus();
        return success(status);
    }

    @PostMapping("/cmd_vel")
    public ResponseEntity<ResponseDTO<String>> sendCmdVel(@Valid @RequestBody TwistDTO dto) {
        // #region agent log
        try {
            String logLine = String.format("{\"id\":\"%s\",\"timestamp\":%d,\"location\":\"TurtlesimController.sendCmdVel:31\",\"message\":\"Entry\",\"data\":{\"linear\":%f,\"angular\":%f},\"sessionId\":\"debug-session\",\"runId\":\"run1\",\"hypothesisId\":\"A\"}\n",
                UUID.randomUUID(), System.currentTimeMillis(), dto.getLinear(), dto.getAngular());
            String logDir = System.getenv().getOrDefault("DEBUG_LOG_DIR", System.getProperty("user.dir", "/app"));
            Path logPath = Paths.get(logDir, ".cursor", "debug.log");
            Files.write(logPath, logLine.getBytes(), StandardOpenOption.CREATE, StandardOpenOption.APPEND);
        } catch (Exception ignored) {}
        // #endregion
        log.debug("Received cmd_vel: linear={}, angular={}", dto.getLinear(), dto.getAngular());
        try {
            turtlesimService.sendCmdVel(dto);
            // #region agent log
            try {
                String logLine = String.format("{\"id\":\"%s\",\"timestamp\":%d,\"location\":\"TurtlesimController.sendCmdVel:40\",\"message\":\"Exit success\",\"data\":{},\"sessionId\":\"debug-session\",\"runId\":\"run1\",\"hypothesisId\":\"A\"}\n",
                    UUID.randomUUID(), System.currentTimeMillis());
                String logDir = System.getenv().getOrDefault("DEBUG_LOG_DIR", System.getProperty("user.dir", "/app"));
            Path logPath = Paths.get(logDir, ".cursor", "debug.log");
                Files.write(logPath, logLine.getBytes(), StandardOpenOption.CREATE, StandardOpenOption.APPEND);
            } catch (Exception ignored) {}
            // #endregion
            return success("cmd_vel_sent");
        } catch (Exception e) {
            // #region agent log
            try {
                String logLine = String.format("{\"id\":\"%s\",\"timestamp\":%d,\"location\":\"TurtlesimController.sendCmdVel:48\",\"message\":\"Exception\",\"data\":{\"exception\":\"%s\",\"message\":\"%s\"},\"sessionId\":\"debug-session\",\"runId\":\"run1\",\"hypothesisId\":\"A\"}\n",
                    UUID.randomUUID(), System.currentTimeMillis(), e.getClass().getName(), e.getMessage().replace("\"", "\\\""));
                String logDir = System.getenv().getOrDefault("DEBUG_LOG_DIR", System.getProperty("user.dir", "/app"));
            Path logPath = Paths.get(logDir, ".cursor", "debug.log");
                Files.write(logPath, logLine.getBytes(), StandardOpenOption.CREATE, StandardOpenOption.APPEND);
            } catch (Exception ignored) {}
            // #endregion
            throw e;
        }
    }
}

