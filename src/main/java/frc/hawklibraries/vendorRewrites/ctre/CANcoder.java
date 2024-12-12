package frc.hawklibraries.vendorRewrites.ctre;

import com.ctre.phoenix6.configs.CANcoderConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;

public class CANcoder {
    
    private com.ctre.phoenix6.hardware.CANcoder m_encoder;
    private Rotation2d encoderOffset;

    public CANcoder(int deviceId) {
        this.m_encoder = new com.ctre.phoenix6.hardware.CANcoder(deviceId);
        this.m_encoder.getConfigurator().apply(new CANcoderConfiguration());
    }

    public void setOffset(Rotation2d encoderOffset) {
        this.encoderOffset = encoderOffset;
    }

    public void setPosition(Rotation2d position) {
        this.encoderOffset = getPosition().minus(position);
    }

    public double getVelocity() {
        return this.m_encoder.getVelocity().getValueAsDouble();
    }

    public Rotation2d getPositionSinceBoot() {
        return Rotation2d.fromRotations(this.m_encoder.getPositionSinceBoot().getValueAsDouble());
    }

    public Rotation2d getPosition() {
        return Rotation2d.fromRotations(this.m_encoder.getPosition().getValueAsDouble()).plus(this.encoderOffset);
    }

}
