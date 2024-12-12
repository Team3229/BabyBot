// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.hawklibraries.vendorRewrites.wpilib.ChassisSpeeds;

public class RobotContainer {
  
  private Drivetrain swerve;

  public RobotContainer() {
    swerve = new Drivetrain(() -> {
      return new ChassisSpeeds(
        0.0,
        0.0,
        0.0
      );
    });
    configureBindings();
  }

  private void configureBindings() {
    
  }
}
