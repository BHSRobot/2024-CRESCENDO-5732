// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter_Box;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterBox extends SubsystemBase {
  private CANSparkMax ShooterNeo1;
  private CANSparkMax ShooterNeo2;

  private ShooterState state = ShooterState.DISABLED;

  /** Creates a new Shooter. */
  public ShooterBox() {
    ShooterNeo1 = new CANSparkMax(15, MotorType.kBrushless);
    ShooterNeo2 = new CANSparkMax(16, MotorType.kBrushless);
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
              ShooterNeo2.set(0);
              break;
          case ENABLED:
              ShooterNeo1.set(1);
              ShooterNeo2.set(1);
              break;
      }
  }

  public Command disabledCommand() {
      return runEnd(() -> {
          setShooterState(ShooterState.DISABLED);
      }, () -> {});
  }

  public Command enabledCommand() {
      return runEnd(() -> {
          setShooterState(ShooterState.ENABLED);
      }, () -> {});
  }

  public void setShooterState(ShooterState state) {
    this.state = state;
  }

}