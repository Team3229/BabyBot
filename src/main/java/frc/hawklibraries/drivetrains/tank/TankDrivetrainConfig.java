package frc.hawklibraries.drivetrains.tank;

import frc.hawklibraries.drivetrains.DrivetrainConfig;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.SuperBuilder;

@SuperBuilder
@Getter
@Setter
public class TankDrivetrainConfig extends DrivetrainConfig {
    
    private TankModuleConfig leftConfig;
    private TankModuleConfig rightConfig;

    private double trackWidthMeters;

}