package com.samma.rcp.app.mapper;

import com.samma.rcp.app.domain.entity.RobotConfig;
import com.samma.rcp.app.dto.RobotConfigDTO;
import javax.annotation.processing.Generated;
import org.springframework.stereotype.Component;

@Generated(
    value = "org.mapstruct.ap.MappingProcessor",
    date = "2026-01-09T12:45:52+0300",
    comments = "version: 1.5.5.Final, compiler: Eclipse JDT (IDE) 3.45.0.v20260101-2150, environment: Java 21.0.9 (Eclipse Adoptium)"
)
@Component
public class RobotConfigMapperImpl implements RobotConfigMapper {

    @Override
    public RobotConfigDTO toDto(RobotConfig e) {
        if ( e == null ) {
            return null;
        }

        RobotConfigDTO.RobotConfigDTOBuilder robotConfigDTO = RobotConfigDTO.builder();

        robotConfigDTO.id( e.getId() );
        robotConfigDTO.model( e.getModel() );

        return robotConfigDTO.build();
    }

    @Override
    public RobotConfig toEntity(RobotConfigDTO d) {
        if ( d == null ) {
            return null;
        }

        RobotConfig.RobotConfigBuilder robotConfig = RobotConfig.builder();

        robotConfig.id( d.getId() );
        robotConfig.model( d.getModel() );

        return robotConfig.build();
    }
}
