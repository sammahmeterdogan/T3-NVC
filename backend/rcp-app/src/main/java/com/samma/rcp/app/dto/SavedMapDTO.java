package com.samma.rcp.app.dto;

import lombok.*;
import java.time.LocalDateTime;

@Data @Builder @NoArgsConstructor @AllArgsConstructor
public class SavedMapDTO {
    private Long id;
    private String name;
    private String scenario; // SLAM scenario used to create this map
    private String filePath;
    private String pgmFilePath;
    private String yamlFilePath;
    private Double sizeMb;
    private Double resolution;
    private Integer width;
    private Integer height;
    private LocalDateTime createdAt;
    private LocalDateTime updatedAt;
}

