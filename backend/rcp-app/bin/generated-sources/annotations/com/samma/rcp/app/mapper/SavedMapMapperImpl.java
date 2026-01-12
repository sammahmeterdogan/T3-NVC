package com.samma.rcp.app.mapper;

import com.samma.rcp.app.domain.entity.SavedMap;
import com.samma.rcp.app.dto.SavedMapDTO;
import javax.annotation.processing.Generated;
import org.springframework.stereotype.Component;

@Generated(
    value = "org.mapstruct.ap.MappingProcessor",
    date = "2026-01-09T12:45:52+0300",
    comments = "version: 1.5.5.Final, compiler: Eclipse JDT (IDE) 3.45.0.v20260101-2150, environment: Java 21.0.9 (Eclipse Adoptium)"
)
@Component
public class SavedMapMapperImpl implements SavedMapMapper {

    @Override
    public SavedMapDTO toDto(SavedMap e) {
        if ( e == null ) {
            return null;
        }

        SavedMapDTO.SavedMapDTOBuilder savedMapDTO = SavedMapDTO.builder();

        savedMapDTO.createdAt( e.getCreatedAt() );
        savedMapDTO.filePath( e.getFilePath() );
        savedMapDTO.height( e.getHeight() );
        savedMapDTO.id( e.getId() );
        savedMapDTO.name( e.getName() );
        savedMapDTO.pgmFilePath( e.getPgmFilePath() );
        savedMapDTO.resolution( e.getResolution() );
        savedMapDTO.scenario( e.getScenario() );
        savedMapDTO.sizeMb( e.getSizeMb() );
        savedMapDTO.updatedAt( e.getUpdatedAt() );
        savedMapDTO.width( e.getWidth() );
        savedMapDTO.yamlFilePath( e.getYamlFilePath() );

        return savedMapDTO.build();
    }

    @Override
    public SavedMap toEntity(SavedMapDTO d) {
        if ( d == null ) {
            return null;
        }

        SavedMap.SavedMapBuilder savedMap = SavedMap.builder();

        savedMap.createdAt( d.getCreatedAt() );
        savedMap.filePath( d.getFilePath() );
        savedMap.height( d.getHeight() );
        savedMap.id( d.getId() );
        savedMap.name( d.getName() );
        savedMap.pgmFilePath( d.getPgmFilePath() );
        savedMap.resolution( d.getResolution() );
        savedMap.scenario( d.getScenario() );
        savedMap.sizeMb( d.getSizeMb() );
        savedMap.updatedAt( d.getUpdatedAt() );
        savedMap.width( d.getWidth() );
        savedMap.yamlFilePath( d.getYamlFilePath() );

        return savedMap.build();
    }
}
