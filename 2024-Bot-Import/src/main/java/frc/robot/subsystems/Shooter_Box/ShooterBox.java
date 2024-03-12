// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter_Box;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.Constants;
import frc.utils.Constants.MechConstants;

public class ShooterBox extends SubsystemBase {
  private static CANSparkMax ShooterNeo1;
  private static CANSparkMax ShooterNeo2;

  private ShooterState state = ShooterState.DISABLED;

  /** Creates a new Shooter. */
  public ShooterBox() {
    ShooterNeo1 = new CANSparkMax(MechConstants.kWristNEOTopID, MotorType.kBrushless);
    ShooterNeo2 = new CANSparkMax(MechConstants.kWristNEOBottomID, MotorType.kBrushless);

    ShooterNeo1.setIdleMode(IdleMode.kBrake);
    ShooterNeo2.setIdleMode(IdleMode.kBrake);

    ShooterNeo2.follow(ShooterNeo1);
  }

  public enum ShooterState {
    ENABLED,
    DISABLED
  }

  public void periodic() {
      //Roller moves 4 times as fast as chamber
      switch (state) {
          case DISABLED:
              ShooterNeo1.set(0);
              break;
          case ENABLED:
              ShooterNeo1.set(-1);
              break;
      }
  }

  public Command disabledCommand() {
      return runOnce(() -> {
          setShooterState(ShooterState.DISABLED);
      });
  }

  public Command enabledCommand() {
      return runOnce(() -> {
          setShooterState(ShooterState.ENABLED);
      });
  }

  public void setShooterState(ShooterState state) {
    this.state = state;
  }
  
  public void setShooterSpeed(double output) {
    ShooterNeo1.set(output);
  }

}