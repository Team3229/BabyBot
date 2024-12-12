package frc.hawklibraries.drivetrains.tank;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.hawklibraries.utilities.PIDConstants;
import lombok.Builder;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.SuperBuilder;

@SuperBuilder
@Getter
@Setter
public class TankModuleConfig {
    
    @Builder.Default
    private double wheelDiameter = 0.25;
    @Builder.Default
    private double gearRatio = 1.0;
    @Builder.Default
    private double maxSpeed = 3.0;
    @Builder.Default
    private MotorType motorType = MotorType.kBrushless;

    private int leadMotorID;
    private int followMotorID;
    private PIDConstants pid;

}
