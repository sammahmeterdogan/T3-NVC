package com.samma.rcp.app.orchestration;

import com.samma.rcp.app.domain.model.RobotModel;
import com.samma.rcp.app.domain.model.ScenarioType;
import org.springframework.stereotype.Component;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.Duration;
import java.util.HashMap;
import java.util.Map;

/**
 * Orchestrates Docker Compose lifecycle for ROS simulation stack.
 *
 * Configuration:
 *  - Compose file path:
 *      * JVM: -Dros.compose.file=ros-stack/docker-compose.yml
 *      * ENV: ROS_COMPOSE_FILE=ros-stack/docker-compose.yml
 *    (Default: ros-stack/docker-compose.yml)
 *  - WebSocket port:
 *      * JVM: -Dros.ws.port=9092
 *      * ENV: ROS_WS_PORT=9092
 *    (Default: 9092 - different from root compose's rosbridge on 9091)
 */
@Component
public class SimulationOrchestrator {
    private final DockerService docker;

    private final Path compose = Paths.get(
            System.getProperty("ros.compose.file",
                    System.getenv().getOrDefault("ROS_COMPOSE_FILE", "ros-stack/docker-compose.yml"))
    ).toAbsolutePath();

    private final int wsPort = Integer.parseInt(
            System.getProperty("ros.ws.port",
                    System.getenv().getOrDefault("ROS_WS_PORT", "9092"))
    );

    public SimulationOrchestrator(DockerService docker) {
        this.docker = docker;
    }

    /**
     * Starts simulation with specified model and scenario.
     * Passes TURTLEBOT3_MODEL and scenario as environment variables to Docker Compose.
     */
    public void start(RobotModel model, ScenarioType scenario) {
        Map<String, String> env = new HashMap<>();
        env.put("TURTLEBOT3_MODEL", model.toEnvValue());
        env.put("GAZEBO_WORLD", mapScenarioToWorld(scenario));
        
        docker.composeUp(compose, env);
        
        boolean ok = docker.waitForPort("localhost", wsPort, Duration.ofSeconds(25));
        if (!ok) {
            throw new IllegalStateException("ROSBridge port " + wsPort + " did not become available");
        }
    }

    public void stop() {
        docker.composeDown(compose);
    }

    public boolean isRunning() {
        return docker.waitForPort("localhost", wsPort, Duration.ofSeconds(1));
    }

    public int getWsPort() {
        return wsPort;
    }

    private String mapScenarioToWorld(ScenarioType scenario) {
        return switch (scenario) {
            case TELEOP, SLAM -> "turtlebot3_world";
            case NAVIGATION -> "turtlebot3_house";
            case OBSTACLE_AVOIDANCE -> "turtlebot3_world";
            default -> "turtlebot3_world";
        };
    }
}
