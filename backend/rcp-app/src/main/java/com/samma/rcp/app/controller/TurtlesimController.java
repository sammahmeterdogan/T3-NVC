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

import java.util.Map;

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
        log.debug("Received cmd_vel: linear={}, angular={}", dto.getLinear(), dto.getAngular());
        turtlesimService.sendCmdVel(dto);
        return success("cmd_vel_sent");
    }
}
