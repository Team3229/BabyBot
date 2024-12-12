package frc.hawklibraries.drivetrains.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.hawklibraries.utilities.PIDConstants;
import lombok.Builder;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.SuperBuilder;

@SuperBuilder
@Getter
@Setter
public class SwerveModuleConfig {
    
    @Builder.Default
    private double wheelDiameter = 0.25;
    @Builder.Default
    private double driveGearRatio = 1.0;
    @Builder.Default
    private double turningGearRatio = 1.0;
    @Builder.Default
    private double maxSpeed = 3.0;
    
    private int driveID;
    private int turningID;
    private int turningEncoderID;
    private Rotation2d encoderOffset;
    private PIDConstants drivePID;
    private PIDConstants turningPID;
    private double turningOutputMin;
    private double turningOutputMax;

}