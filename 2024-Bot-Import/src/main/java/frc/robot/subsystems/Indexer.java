// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MechConstants;

public class Indexer extends SubsystemBase {
  private CANSparkMax indexNEO;

  private IndexState state = IndexState.DISABLED;


  /** Creates a new Shooter. */
  public Indexer() {
    indexNEO = new CANSparkMax(MechConstants.kIndexerID, MotorType.kBrushless);
    indexNEO.setIdleMode(IdleMode.kBrake);
  }

  public enum IndexState {
    FORWARD,
    BACK,
    DISABLED
  }

  public void periodic() {
      //Roller moves 4 times as fast as chamber
      switch (state) {
          case FORWARD:
              indexNEO.set(.75);
              break;
          case BACK:
              indexNEO.set(-.75);
              break;
          case DISABLED:
              indexNEO.set(0);
              break;
      }
  }

  public Command disabledCommand() {
      return runEnd(() -> {
          setIndexState(IndexState.DISABLED);
      }, () -> {});
  }

  public Command forwardCommand() {
      return runEnd(() -> {
          setIndexState(IndexState.FORWARD);
      }, () -> {});
  }

  public Command backCommand() {
      return runEnd(() -> {
          setIndexState(IndexState.BACK);
      }, () -> {});
  }

  public void setIndexState(IndexState state) {
    this.state = state;
  }

}