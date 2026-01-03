package com.samma.rcp.app.domain.entity;

import jakarta.persistence.*;
import lombok.*;

import java.time.LocalDateTime;

@Entity
@Table(name = "saved_map")
@Data @Builder @NoArgsConstructor @AllArgsConstructor
public class SavedMap {
    @Id @GeneratedValue(strategy = GenerationType.IDENTITY) private Long id;

    @Column(nullable=false) private String name;
    @Column(length=100) private String scenario; // SLAM, CARTOGRAPHER, etc.
    @Column(name="file_path", nullable=false, length=500) private String filePath;
    @Column(name="pgm_file_path", length=500) private String pgmFilePath;
    @Column(name="yaml_file_path", length=500) private String yamlFilePath;

    @Column(name="size_mb") private Double sizeMb;
    private Double resolution;
    private Integer width;
    private Integer height;

    @Column(name="created_at", nullable=false) private LocalDateTime createdAt;
    @Column(name="updated_at") private LocalDateTime updatedAt;

    @PrePersist protected void onCreate(){ createdAt = LocalDateTime.now(); }
    @PreUpdate  protected void onUpdate(){ updatedAt = LocalDateTime.now(); }
}
