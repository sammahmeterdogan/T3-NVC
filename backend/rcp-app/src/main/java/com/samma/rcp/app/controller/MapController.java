package com.samma.rcp.app.controller;

import com.samma.rcp.app.dto.SavedMapDTO;
import com.samma.rcp.app.service.MapService;
import com.samma.rcp.base.controller.BaseController;
import com.samma.rcp.base.dto.ResponseDTO;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.http.HttpHeaders;
import org.springframework.http.HttpStatus;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;
import org.springframework.web.multipart.MultipartFile;

import java.util.*;

/**
 * Map Controller - Complete CRUD for SLAM maps
 */
@Slf4j
@RestController
@RequestMapping("/api/map")
@RequiredArgsConstructor
public class MapController extends BaseController {

    private final MapService mapService;

    /**
     * Save current map from SLAM
     */
    @PostMapping("/save")
    public ResponseEntity<ResponseDTO<SavedMapDTO>> save(@RequestBody Map<String, String> body) {
        String name = body.getOrDefault("name", "map_" + System.currentTimeMillis());
        String scenario = body.getOrDefault("scenario", "UNKNOWN");
        
        log.info("[MapController] Saving map: {}, scenario: {}", name, scenario);
        
        try {
            SavedMapDTO saved = mapService.saveMap(name, scenario);
            return created(saved);
        } catch (Exception e) {
            log.error("[MapController] Failed to save map", e);
            return error("Failed to save map: " + e.getMessage(), HttpStatus.INTERNAL_SERVER_ERROR);
        }
    }

    /**
     * List all saved maps
     */
    @GetMapping("/list")
    public ResponseEntity<ResponseDTO<List<SavedMapDTO>>> list() {
        try {
            List<SavedMapDTO> maps = mapService.listMaps();
            return success(maps);
        } catch (Exception e) {
            log.error("[MapController] Failed to list maps", e);
            return error("Failed to list maps: " + e.getMessage(), HttpStatus.INTERNAL_SERVER_ERROR);
        }
    }

    /**
     * Get map by ID
     */
    @GetMapping("/{id}")
    public ResponseEntity<ResponseDTO<SavedMapDTO>> getById(@PathVariable Long id) {
        try {
            SavedMapDTO map = mapService.getMapById(id);
            return success(map);
        } catch (Exception e) {
            log.error("[MapController] Failed to get map {}", id, e);
            return error("Map not found", HttpStatus.NOT_FOUND);
        }
    }

    /**
     * Load map for navigation
     */
    @PostMapping("/load/{id}")
    public ResponseEntity<ResponseDTO<String>> load(@PathVariable Long id) {
        try {
            log.info("[MapController] Loading map: {}", id);
            mapService.loadMap(id);
            return success("Map loaded successfully");
        } catch (Exception e) {
            log.error("[MapController] Failed to load map {}", id, e);
            return error("Failed to load map: " + e.getMessage());
        }
    }

    /**
     * Delete map
     */
    @DeleteMapping("/{id}")
    public ResponseEntity<ResponseDTO<String>> delete(@PathVariable Long id) {
        try {
            log.info("[MapController] Deleting map: {}", id);
            mapService.deleteMap(id);
            return success("Map deleted successfully");
        } catch (Exception e) {
            log.error("[MapController] Failed to delete map {}", id, e);
            return error("Failed to delete map: " + e.getMessage());
        }
    }

    /**
     * Download map files (PGM + YAML as ZIP)
     */
    @GetMapping("/download/{id}")
    public ResponseEntity<byte[]> download(@PathVariable Long id) {
        try {
            log.info("[MapController] Downloading map: {}", id);
            byte[] zipData = mapService.downloadMap(id);
            
            HttpHeaders headers = new HttpHeaders();
            headers.setContentType(MediaType.APPLICATION_OCTET_STREAM);
            headers.setContentDispositionFormData("attachment", "map_" + id + ".zip");
            
            return ResponseEntity.ok()
                    .headers(headers)
                    .body(zipData);
                    
        } catch (Exception e) {
            log.error("[MapController] Failed to download map {}", id, e);
            return ResponseEntity.notFound().build();
        }
    }

    /**
     * Upload map (PGM + YAML files)
     */
    @PostMapping("/upload")
    public ResponseEntity<ResponseDTO<SavedMapDTO>> upload(
            @RequestParam("name") String name,
            @RequestParam("pgm") MultipartFile pgmFile,
            @RequestParam("yaml") MultipartFile yamlFile) {
        
        try {
            log.info("[MapController] Uploading map: {}", name);
            SavedMapDTO uploaded = mapService.uploadMap(name, pgmFile, yamlFile);
            return created(uploaded);
        } catch (Exception e) {
            log.error("[MapController] Failed to upload map", e);
            return error("Failed to upload map: " + e.getMessage());
        }
    }

    /**
     * Get map preview image (PGM as PNG)
     */
    @GetMapping("/preview/{id}")
    public ResponseEntity<byte[]> getPreview(@PathVariable Long id) {
        try {
            byte[] preview = mapService.getMapPreview(id);
            
            HttpHeaders headers = new HttpHeaders();
            headers.setContentType(MediaType.IMAGE_PNG);
            
            return ResponseEntity.ok()
                    .headers(headers)
                    .body(preview);
                    
        } catch (Exception e) {
            log.warn("[MapController] Failed to get preview for map {}", id);
            return ResponseEntity.notFound().build();
        }
    }
}
