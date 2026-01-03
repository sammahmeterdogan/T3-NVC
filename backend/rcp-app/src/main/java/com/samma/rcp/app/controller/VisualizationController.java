package com.samma.rcp.app.controller;

import com.samma.rcp.app.config.RosProperties;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import java.util.Map;

@RestController
@RequestMapping("/api/visualization")
public class VisualizationController {

    private final RosProperties rosProperties;

    public VisualizationController(RosProperties rosProperties) {
        this.rosProperties = rosProperties;
        // Log resolved URLs at startup for observability
        String rviz = rosProperties.getVisualization().getRvizUrl();
        String turtlesim = rosProperties.getVisualization().getTurtlesimUrl();
        System.out.println("[visualization] RViz URL: " + rviz);
        System.out.println("[visualization] Turtlesim URL: " + turtlesim);
    }

    @GetMapping("/rviz")
    public Map<String, String> getRvizUrl() {
        return Map.of("url", rosProperties.getVisualization().getRvizUrl());
    }

    @GetMapping("/turtlesim")
    public Map<String, String> getTurtlesimUrl() {
        return Map.of("url", rosProperties.getVisualization().getTurtlesimUrl());
    }
}

