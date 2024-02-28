// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter_Box;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.MechConstants;

public class ShooterBoxPivot extends ProfiledPIDSubsystem {
  /** Creates a new ShooterBoxPivot. */
  public ShooterBoxPivot() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            MechConstants.kPElevAngle,
            MechConstants.kIElevAngle,
            MechConstants.kDElevAngle,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(.3, .05)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
    Logger.recordOutput("Wrist Pivot Angle: ", getMeasurement());
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
}
