package com.samma.rcp.app.service;

import com.samma.rcp.app.domain.entity.SavedMap;
import com.samma.rcp.app.domain.repo.SavedMapRepository;
import com.samma.rcp.app.dto.SavedMapDTO;
import com.samma.rcp.app.mapper.SavedMapMapper;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Service;
import org.springframework.web.multipart.MultipartFile;

import java.io.ByteArrayOutputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.stream.Collectors;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

/**
 * Map Service - Complete CRUD for SLAM maps
 */
@Slf4j
@Service
@RequiredArgsConstructor
public class MapService {

    private final SavedMapRepository repo;
    private final SavedMapMapper mapper;

    /**
     * Save current SLAM map
     */
    public SavedMapDTO saveMap(String name, String scenario) {
        try {
            Path mapsDir = Path.of("ros-stack", "maps");
            Files.createDirectories(mapsDir);
            Path yaml = mapsDir.resolve(name + ".yaml");
            Path pgm = mapsDir.resolve(name + ".pgm");
            
            // Create placeholder files (in production, copy from SLAM output)
            Files.writeString(yaml, "# SLAM Map: " + name + "\nimage: " + name + ".pgm\nresolution: 0.05\norigin: [0.0,0.0,0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n");
            if (!Files.exists(pgm)) Files.createFile(pgm);

            SavedMap m = repo.save(SavedMap.builder()
                    .name(name)
                    .scenario(scenario)
                    .filePath(mapsDir.toAbsolutePath().toString())
                    .yamlFilePath(yaml.toAbsolutePath().toString())
                    .pgmFilePath(pgm.toAbsolutePath().toString())
                    .sizeMb(Files.size(pgm) / 1024d / 1024d)
                    .resolution(0.05)
                    .width(0)
                    .height(0)
                    .build());

            log.info("[MapService] Map saved: {} (ID: {})", name, m.getId());
            return mapper.toDto(m);
            
        } catch (Exception e) {
            log.error("[MapService] Failed to save map: {}", name, e);
            throw new RuntimeException("Map save failed: " + e.getMessage(), e);
        }
    }

    /**
     * List all maps
     */
    public List<SavedMapDTO> listMaps() {
        try {
            return repo.findAll().stream()
                    .map(mapper::toDto)
                    .collect(Collectors.toList());
        } catch (Exception e) {
            log.error("[MapService] Failed to list maps", e);
            throw new RuntimeException("Failed to list maps: " + e.getMessage(), e);
        }
    }

    /**
     * Get map by ID
     */
    public SavedMapDTO getMapById(Long id) {
        SavedMap map = repo.findById(id)
                .orElseThrow(() -> new RuntimeException("Map not found: " + id));
        return mapper.toDto(map);
    }

    /**
     * Load map for navigation (publish to map_server)
     */
    public void loadMap(Long id) {
        try {
            SavedMap map = repo.findById(id)
                    .orElseThrow(() -> new RuntimeException("Map not found: " + id));
            
            log.info("[MapService] Loading map: {} (ID: {})", map.getName(), id);
            
            // In production: publish to map_server or call ROS service
            // For now, just validate files exist
            Path yamlPath = Path.of(map.getYamlFilePath());
            Path pgmPath = Path.of(map.getPgmFilePath());
            
            if (!Files.exists(yamlPath)) {
                throw new RuntimeException("YAML file not found: " + yamlPath);
            }
            if (!Files.exists(pgmPath)) {
                throw new RuntimeException("PGM file not found: " + pgmPath);
            }
            
            log.info("[MapService] Map loaded successfully: {}", map.getName());
            // TODO: Publish to /map_server/load_map service or update Nav2 configuration
            
        } catch (Exception e) {
            log.error("[MapService] Failed to load map {}", id, e);
            throw new RuntimeException("Failed to load map: " + e.getMessage(), e);
        }
    }

    /**
     * Delete map
     */
    public void deleteMap(Long id) {
        try {
            SavedMap map = repo.findById(id)
                    .orElseThrow(() -> new RuntimeException("Map not found: " + id));
            
            log.info("[MapService] Deleting map: {} (ID: {})", map.getName(), id);
            
            // Delete files
            try {
                if (map.getYamlFilePath() != null) {
                    Files.deleteIfExists(Path.of(map.getYamlFilePath()));
                }
                if (map.getPgmFilePath() != null) {
                    Files.deleteIfExists(Path.of(map.getPgmFilePath()));
                }
            } catch (Exception e) {
                log.warn("[MapService] Failed to delete map files", e);
            }
            
            // Delete from database
            repo.deleteById(id);
            log.info("[MapService] Map deleted: {}", map.getName());
            
        } catch (Exception e) {
            log.error("[MapService] Failed to delete map {}", id, e);
            throw new RuntimeException("Failed to delete map: " + e.getMessage(), e);
        }
    }

    /**
     * Download map as ZIP (PGM + YAML)
     */
    public byte[] downloadMap(Long id) {
        try {
            SavedMap map = repo.findById(id)
                    .orElseThrow(() -> new RuntimeException("Map not found: " + id));
            
            log.info("[MapService] Downloading map: {} (ID: {})", map.getName(), id);
            
            ByteArrayOutputStream baos = new ByteArrayOutputStream();
            try (ZipOutputStream zos = new ZipOutputStream(baos)) {
                // Add YAML file
                if (map.getYamlFilePath() != null) {
                    Path yamlPath = Path.of(map.getYamlFilePath());
                    if (Files.exists(yamlPath)) {
                        ZipEntry yamlEntry = new ZipEntry(map.getName() + ".yaml");
                        zos.putNextEntry(yamlEntry);
                        Files.copy(yamlPath, zos);
                        zos.closeEntry();
                    }
                }
                
                // Add PGM file
                if (map.getPgmFilePath() != null) {
                    Path pgmPath = Path.of(map.getPgmFilePath());
                    if (Files.exists(pgmPath)) {
                        ZipEntry pgmEntry = new ZipEntry(map.getName() + ".pgm");
                        zos.putNextEntry(pgmEntry);
                        Files.copy(pgmPath, zos);
                        zos.closeEntry();
                    }
                }
            }
            
            return baos.toByteArray();
            
        } catch (Exception e) {
            log.error("[MapService] Failed to download map {}", id, e);
            throw new RuntimeException("Failed to download map: " + e.getMessage(), e);
        }
    }

    /**
     * Upload map files
     */
    public SavedMapDTO uploadMap(String name, MultipartFile pgmFile, MultipartFile yamlFile) {
        try {
            log.info("[MapService] Uploading map: {}", name);
            
            Path mapsDir = Path.of("ros-stack", "maps");
            Files.createDirectories(mapsDir);
            
            Path pgmPath = mapsDir.resolve(name + ".pgm");
            Path yamlPath = mapsDir.resolve(name + ".yaml");
            
            // Save uploaded files
            pgmFile.transferTo(pgmPath);
            yamlFile.transferTo(yamlPath);
            
            // Create database entry
            SavedMap map = repo.save(SavedMap.builder()
                    .name(name)
                    .filePath(mapsDir.toAbsolutePath().toString())
                    .pgmFilePath(pgmPath.toAbsolutePath().toString())
                    .yamlFilePath(yamlPath.toAbsolutePath().toString())
                    .sizeMb(Files.size(pgmPath) / 1024d / 1024d)
                    .build());
            
            log.info("[MapService] Map uploaded: {} (ID: {})", name, map.getId());
            return mapper.toDto(map);
            
        } catch (Exception e) {
            log.error("[MapService] Failed to upload map: {}", name, e);
            throw new RuntimeException("Failed to upload map: " + e.getMessage(), e);
        }
    }

    /**
     * Get map preview as PNG (convert PGM if needed)
     */
    public byte[] getMapPreview(Long id) {
        try {
            SavedMap map = repo.findById(id)
                    .orElseThrow(() -> new RuntimeException("Map not found: " + id));
            
            Path pgmPath = Path.of(map.getPgmFilePath());
            
            if (!Files.exists(pgmPath)) {
                throw new RuntimeException("PGM file not found");
            }
            
            // For now, return PGM as-is (browsers can render simple PGM)
            // In production, convert to PNG using ImageIO
            return Files.readAllBytes(pgmPath);
            
        } catch (Exception e) {
            log.error("[MapService] Failed to get preview for map {}", id, e);
            throw new RuntimeException("Failed to get preview: " + e.getMessage(), e);
        }
    }
}
