package com.samma.rcp.app.service;

import com.samma.rcp.app.dto.SimStatusDto;
import com.samma.rcp.app.domain.model.RobotModel;
import com.samma.rcp.app.domain.model.ScenarioType;
import com.samma.rcp.app.dto.SimulationStartRequest;
import com.samma.rcp.app.orchestration.SimulationOrchestrator;
import org.springframework.stereotype.Service;

/**
 * Service layer bridging Controller and Orchestrator.
 * Returns SimStatusDto for consistent API responses.
 */
@Service
public class SimulationService {

    private final SimulationOrchestrator orchestrator;

    public SimulationService(SimulationOrchestrator orchestrator) {
        this.orchestrator = orchestrator;
    }

    /** Starts simulation with optional model/scenario parameters. */
    public SimStatusDto start(SimulationStartRequest request) {
        RobotModel model = (request != null && request.getModel() != null) 
            ? request.getModel() 
            : RobotModel.BURGER;
        ScenarioType scenario = (request != null && request.getScenario() != null) 
            ? request.getScenario() 
            : ScenarioType.TELEOP;
        
        orchestrator.start(model, scenario);
        return buildStatus();
    }

    /** Simülasyonu durdurur ve güncel durumu döner. */
    public SimStatusDto stop() {
        orchestrator.stop();
        return buildStatus();
    }

    /** Anlık durumu döner. */
    public SimStatusDto status() {
        return buildStatus();
    }

    // ---- helper ----
    private SimStatusDto buildStatus() {
        boolean running = orchestrator.isRunning();
        int port = orchestrator.getWsPort(); // Orchestrator’ında hazır.
        // Ortam değişkeniyle host override edilebilir; yoksa localhost.
        String host = System.getenv().getOrDefault("ROSBRIDGE_HOST", "localhost");
        String wsUrl = "ws://" + host + ":" + port;
        return new SimStatusDto(running, wsUrl, port);
    }
}
