package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.hawklibraries.drivetrains.swerve.SwerveDrivetrain;
import frc.hawklibraries.drivetrains.swerve.SwerveDrivetrainConfig;
import frc.hawklibraries.drivetrains.swerve.SwerveModuleConfig;
import frc.hawklibraries.drivetrains.swerve.SwerveModuleConfig.SwerveModuleConfigBuilder;
import frc.hawklibraries.utilities.PIDConstants;
import frc.hawklibraries.vendorRewrites.wpilib.ChassisSpeeds;

public class Drivetrain extends SwerveDrivetrain {

    private static SwerveModuleConfigBuilder<?,?> getBaseModuleConfig() {
        return SwerveModuleConfig.builder()
            .driveGearRatio(1)
            .drivePID(new PIDConstants(1, 0, 0, 0))
            .turningGearRatio(1)
            .turningPID(new PIDConstants(0, 0, 0, 0))
            .turningOutputMin(-0.75)
            .turningOutputMax(0.75)
            .wheelDiameter(0)
            .maxSpeed(0);
    }

    public Drivetrain(Supplier<ChassisSpeeds> defaultDrivingSpeeds) {
        super(
            SwerveDrivetrainConfig.builder()
                .maxSpeed(3)
                .maxAccel(1)
                .maxTurnRate(2*Math.PI)
                .moduleDistance(0.35)
                .robotLength(0.483)
                .robotWidth(0.483)
                .rotationPID(new PIDConstants(0))
                .translationPID(new PIDConstants(0))
                .frontLeftConfig(
                    getBaseModuleConfig()
                        .driveID(0)
                        .turningID(1)
                        .turningEncoderID(0)
                        .encoderOffset(new Rotation2d())
                    .build()
                )
                .frontRightConfig(
                    getBaseModuleConfig()
                        .driveID(2)
                        .turningID(3)
                        .turningEncoderID(0)
                        .encoderOffset(new Rotation2d())
                    .build()
                )
                .backLeftConfig(
                    getBaseModuleConfig()
                        .driveID(4)
                        .turningID(5)
                        .turningEncoderID(0)
                        .encoderOffset(new Rotation2d())
                    .build()
                )
                .frontRightConfig(
                    getBaseModuleConfig()
                        .driveID(6)
                        .turningID(7)
                        .turningEncoderID(0)
                        .encoderOffset(new Rotation2d())
                    .build()
                )
            .build(),
            defaultDrivingSpeeds
        );
    }
}
