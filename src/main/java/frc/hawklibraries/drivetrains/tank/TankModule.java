package frc.hawklibraries.drivetrains.tank;

import com.revrobotics.CANSparkBase.IdleMode;

import frc.hawklibraries.vendorRewrites.rev.CANSparkMax;
import frc.hawklibraries.vendorRewrites.rev.CANSparkMax.SetpointType;
import frc.hawklibraries.vendorRewrites.wpilib.PIDController;

public class TankModule {
    
    TankModuleConfig config;

    CANSparkMax leadMotor;
    CANSparkMax followMotor;

    PIDController pidController;

    public TankModule(TankModuleConfig config) {
        
        this.config = config;

        leadMotor = new CANSparkMax(config.getLeadMotorID(), config.getMotorType());
        followMotor = new CANSparkMax(config.getFollowMotorID(), config.getMotorType());

        pidController = new PIDController(config.getPid());

        leadMotor.restoreFactoryDefaults();
        followMotor.restoreFactoryDefaults();
        
        leadMotor.setIdleMode(IdleMode.kCoast);
        followMotor.setIdleMode(IdleMode.kCoast);
        leadMotor.setPositionConversionFactor(Math.PI * config.getWheelDiameter() / config.getGearRatio());
        leadMotor.setVelocityConversionFactor(Math.PI * config.getWheelDiameter() / 60 / config.getGearRatio());
        leadMotor.setPosition(0);
        followMotor.setPositionConversionFactor(Math.PI * config.getWheelDiameter() / config.getGearRatio());
        followMotor.setVelocityConversionFactor(Math.PI * config.getWheelDiameter() / 60 / config.getGearRatio());
        followMotor.setPosition(0);

        followMotor.follow(leadMotor, true);

        pidController.applyToSparkMax(leadMotor, SetpointType.kVelocity);

    }

    public void setDesiredState(double speedMetersPerSecond) {
        pidController.setSetpoint(speedMetersPerSecond);
    }

    public double getDistanceMeters() {
        return leadMotor.getPosition();
    }

}
