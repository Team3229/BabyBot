package frc.hawklibraries.drivetrains.swerve;

import frc.hawklibraries.drivetrains.DrivetrainConfig;
import frc.hawklibraries.utilities.PIDConstants;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.SuperBuilder;

@SuperBuilder
@Getter
@Setter
public class SwerveDrivetrainConfig extends DrivetrainConfig {

    private double[] moduleOffsets;
    private double moduleDistance;
    private PIDConstants translationPID;
    private PIDConstants rotationPID;
    private SwerveModuleConfig frontLeftConfig;
    private SwerveModuleConfig frontRightConfig;
    private SwerveModuleConfig backLeftConfig;
    private SwerveModuleConfig backRightConfig;

}