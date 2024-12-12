package frc.hawklibraries.drivetrains.swerve;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.hawklibraries.vendorRewrites.ctre.CANcoder;
import frc.hawklibraries.vendorRewrites.rev.CANSparkMax;
import frc.hawklibraries.vendorRewrites.rev.CANSparkMax.SetpointType;
import frc.hawklibraries.vendorRewrites.wpilib.PIDController;

public class SwerveModule {

    SwerveModuleConfig config;

    CANSparkMax m_driveMotor;
    CANSparkMax m_turningMotor;
    CANcoder m_turningEncoder;

    PIDController m_drivePIDController;
    PIDController m_turningPIDController;
    
    public SwerveModule(SwerveModuleConfig config) {

        this.config = config;

        m_driveMotor = new CANSparkMax(this.config.getDriveID(), MotorType.kBrushless);
        m_turningMotor = new CANSparkMax(this.config.getTurningID(), MotorType.kBrushless);
        m_turningEncoder = new CANcoder(this.config.getTurningEncoderID());
        m_drivePIDController = new PIDController(this.config.getDrivePID());
        m_turningPIDController = new PIDController(this.config.getTurningPID());

        m_driveMotor.restoreFactoryDefaults();
        m_turningMotor.restoreFactoryDefaults();

        m_driveMotor.setIdleMode(IdleMode.kCoast);
        m_turningMotor.setIdleMode(IdleMode.kCoast);
        m_driveMotor.setPositionConversionFactor(Math.PI * config.getWheelDiameter() / config.getDriveGearRatio());
        m_driveMotor.setVelocityConversionFactor(Math.PI * config.getWheelDiameter() / 60 / config.getDriveGearRatio());
        m_driveMotor.setPosition(0);
        m_turningMotor.setPositionConversionFactor(2 * Math.PI / config.getDriveGearRatio());
        m_turningMotor.setInverted(true);

        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        m_drivePIDController.applyToSparkMax(m_driveMotor, SetpointType.kVelocity);
        m_turningPIDController.applyToSparkMax(m_turningMotor, SetpointType.kPosition);

        m_turningEncoder.setOffset(this.config.getEncoderOffset());

        seedSparkMax();

    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition();
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d currentRotation = new Rotation2d(m_turningMotor.getPosition());

        // Optimize the desired state to minimize rotation
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, currentRotation);

        // Scale speed to adjust for angle error
        optimizedState.speedMetersPerSecond *= optimizedState.angle.minus(currentRotation).getCos();

        // Set the desired speed for the drive motor
        m_drivePIDController.setSetpoint(optimizedState.speedMetersPerSecond);

        // Set the desired angle for the turning motor
        m_turningPIDController.setSetpoint(optimizedState.angle.getRadians());
    }

    private void seedSparkMax() {
        m_turningMotor.setPosition(m_turningEncoder.getPosition().getRadians());
    }

}
