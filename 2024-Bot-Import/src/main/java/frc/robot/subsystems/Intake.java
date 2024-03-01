// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MechConstants;

public class Intake extends SubsystemBase {
  private CANSparkMax neo1;

  private IntakeState state = IntakeState.DISABLED;


  /** Creates a new Shooter. */
  public Intake() {
    neo1 = new CANSparkMax(MechConstants.kIntakeID, MotorType.kBrushless);
    neo1.setIdleMode(IdleMode.kBrake);
  }

  public enum IntakeState {
    INTAKING,
    EJECTING,
    DISABLED
  }

  public void periodic() {
      //Roller moves 4 times as fast as chamber
      switch (state) {
          case DISABLED:
              neo1.set(0);
              break;
          case EJECTING:
              neo1.set(1);
              break;
          case INTAKING:
              neo1.set(-1);
              break;
      }
  }

  public Command disabledCommand() {
      return runEnd(() -> {
          setIntakeState(IntakeState.DISABLED);
      }, () -> {});
  }

  public Command ejectCommand() {
      return runEnd(() -> {
          setIntakeState(IntakeState.EJECTING);
      }, () -> {});
  }

  public Command intakeCommand() {
      return runEnd(() -> {
          setIntakeState(IntakeState.INTAKING);
      }, () -> {});
  }

  public void setIntakeState(IntakeState state) {
    this.state = state;
  }

}