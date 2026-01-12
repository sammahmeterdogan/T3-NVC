package com.samma.rcp.app.service;

import com.samma.rcp.app.dto.TwistDTO;
import com.samma.rcp.app.orchestration.RosBridgeClient;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;

import java.util.HashMap;
import java.util.Map;

@Slf4j
@Service
@RequiredArgsConstructor
public class TurtlesimService {

    private final RosBridgeClient rosBridgeClient;

    @Value("${ros.turtlesim.topic:/turtle1/cmd_vel}")
    private String turtlesimTopic;

    public void sendCmdVel(TwistDTO dto) {
        TwistDTO turtlesimDto = TwistDTO.builder()
                .linear(dto.getLinear())
                .angular(dto.getAngular())
                .topic(turtlesimTopic)
                .build();
        
        log.debug("Publishing to {}: linear={}, angular={}", turtlesimTopic, dto.getLinear(), dto.getAngular());
        rosBridgeClient.publishTwist(turtlesimDto);
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
