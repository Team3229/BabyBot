package frc.hawklibraries.drivetrains;

import lombok.Builder;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.SuperBuilder;

@SuperBuilder
@Getter
@Setter
public class DrivetrainConfig {

    @Builder.Default
    private double maxSpeed = 3.0;
    @Builder.Default
    private double maxAccel = 1.0;
    @Builder.Default
    private double maxTurnRate = 1.0;
    @Builder.Default
    private double robotWidth = 1.0;
    @Builder.Default
    private double robotLength = 1.0;

}