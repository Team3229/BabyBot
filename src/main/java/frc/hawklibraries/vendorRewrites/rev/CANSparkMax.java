package frc.hawklibraries.vendorRewrites.rev;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.hawklibraries.utilities.PIDConstants;

public class CANSparkMax {

    public static enum SetpointType {
        kPosition,
        kVelocity
    }
    
    private com.revrobotics.CANSparkMax m_sparkMax;
    private RelativeEncoder m_encoder;
    private SparkPIDController m_pidController;

    private Double currentSetpoint;

    public CANSparkMax(int deviceId, MotorType motorType) {
        this.m_sparkMax = new com.revrobotics.CANSparkMax(deviceId, motorType);
        this.m_encoder = m_sparkMax.getEncoder();
        this.m_pidController = m_sparkMax.getPIDController();
    }

    public void setPID(PIDConstants pid) {
        this.m_pidController.setP(pid.getkP());
        this.m_pidController.setI(pid.getkI());
        this.m_pidController.setD(pid.getkD());
        this.m_pidController.setFF(pid.getkFF());
    }

    public void setSetpoint(double setpoint, SetpointType type) {
        if (type == SetpointType.kPosition) {
            this.currentSetpoint = setpoint;
            this.m_pidController.setReference(setpoint, ControlType.kPosition);
        }

        if (type == SetpointType.kVelocity) {
            this.currentSetpoint = setpoint;
            this.m_pidController.setReference(setpoint, ControlType.kVelocity);
        }
    }

    public void enablePositionWrapping(double min, double max) {
        this.m_pidController.setPositionPIDWrappingMinInput(min);
        this.m_pidController.setPositionPIDWrappingMaxInput(max);
        this.m_pidController.setPositionPIDWrappingEnabled(true);
    }

    public void disablePositionWrapping() {
        this.m_pidController.setPositionPIDWrappingEnabled(false);
    }

    public void set(double speed) {
        this.currentSetpoint = speed;
        this.m_pidController.setReference(speed, ControlType.kDutyCycle);
    }

    public void stopMotor() {
        this.currentSetpoint = null;
        this.m_pidController.setReference(0, ControlType.kDutyCycle);
    }

    public void restoreFactoryDefaults() {
        this.m_sparkMax.restoreFactoryDefaults();
    }

    public Double getSetpoint() {
        return this.currentSetpoint;
    }

    public double getPosition() {
        return this.m_encoder.getPosition();
    }

    public double getVelocity() {
        return this.m_encoder.getVelocity();
    }

    public void setPositionConversionFactor(double conversionFactor) {
        this.m_encoder.setPositionConversionFactor(conversionFactor);
    }

    public void setVelocityConversionFactor(double conversionFactor) {
        this.m_encoder.setVelocityConversionFactor(conversionFactor);
    }

    public void setPosition(double position) {
        this.m_encoder.setPosition(position);
    }

    public void setIdleMode(IdleMode idleMode) {
        this.m_sparkMax.setIdleMode(idleMode);
    }

    public void setInverted(boolean inverted) {
        this.m_sparkMax.setInverted(inverted);
    }

    public void setOutputRange(double min, double max) {
        this.m_pidController.setOutputRange(min, max);
    }

    public void follow(CANSparkMax leader) {
        follow(leader, false);
    }

    public void follow(CANSparkMax leader, boolean invert) {
        m_sparkMax.follow(leader.m_sparkMax, invert);
    }

}
