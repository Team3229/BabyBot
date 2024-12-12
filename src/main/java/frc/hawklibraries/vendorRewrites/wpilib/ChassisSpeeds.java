package frc.hawklibraries.vendorRewrites.wpilib;

import edu.wpi.first.math.geometry.Rotation2d;

public class ChassisSpeeds {
    
    public Double vxMetersPerSecond;
    public Double vyMetersPerSecond;
    public Double omegaRadiansPerSecond;

    public ChassisSpeeds() {}

    public ChassisSpeeds(edu.wpi.first.math.kinematics.ChassisSpeeds speeds) {
        this.vxMetersPerSecond = speeds.vxMetersPerSecond;
        this.vyMetersPerSecond = speeds.vyMetersPerSecond;
        this.omegaRadiansPerSecond = speeds.omegaRadiansPerSecond;
    }

    public ChassisSpeeds(Double vxMetersPerSecond, Double vyMetersPerSecond, Double omegaRadiansPerSecond) {
        this.vxMetersPerSecond = vxMetersPerSecond;
        this.vyMetersPerSecond = vyMetersPerSecond;
        this.omegaRadiansPerSecond = omegaRadiansPerSecond;
    }

    public static ChassisSpeeds fromFieldRelativeSpeeds(ChassisSpeeds fieldRelativeSpeeds, Rotation2d robotAngle) {
        return new ChassisSpeeds(edu.wpi.first.math.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.getChassisSpeeds(), null));
    }

    public edu.wpi.first.math.kinematics.ChassisSpeeds getChassisSpeeds() {
        return new edu.wpi.first.math.kinematics.ChassisSpeeds(
            vxMetersPerSecond == null
                ? 0
                : vxMetersPerSecond,
            vyMetersPerSecond == null
                ? 0
                : vyMetersPerSecond,
            omegaRadiansPerSecond == null
                ? 0
                : omegaRadiansPerSecond
        );
    }

    public static ChassisSpeeds discretize(ChassisSpeeds continuousSpeeds, double dtSeconds) {
        return new ChassisSpeeds(edu.wpi.first.math.kinematics.ChassisSpeeds.discretize(continuousSpeeds.getChassisSpeeds(), dtSeconds));
    }

}
