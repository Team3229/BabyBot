package frc.hawklibraries.utilities;

import java.util.NoSuchElementException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Alliance {

    public static enum AllianceColor {
        Red,
        Blue,
        Null
    }
    
    public static AllianceColor getAlliance() {
        try {
            return wpiToHawk(DriverStation.getAlliance().get());
        } catch (NoSuchElementException e) {
            return AllianceColor.Null;
        }
    }

    public static Trigger alliancePresentTrigger() {
        return new Trigger(
            () -> DriverStation.getAlliance().isPresent()
        );
    }

    private static AllianceColor wpiToHawk(edu.wpi.first.wpilibj.DriverStation.Alliance a) {
        if (a == edu.wpi.first.wpilibj.DriverStation.Alliance.Blue) {
            return AllianceColor.Blue;
        } else {
            return AllianceColor.Red;
        }
    }

}
