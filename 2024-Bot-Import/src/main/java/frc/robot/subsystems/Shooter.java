// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private CANSparkMax neo1;
  private CANSparkMax neo2;

  /** Creates a new Shooter. */
  public Shooter() {
    //neo1 = new CANSparkMax(13, MotorType.kBrushless);
    //neo2 = new CANSparkMax(14, MotorType.kBrushless); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShooter(boolean aButton, boolean bButton) {
      if (aButton && bButton) {
          neo1.set(0);
          neo2.set(0);
      }
      else if (aButton && !bButton) {
          neo1.set(1);
          neo2.set(-1);
      }
      else if (!aButton && bButton) {
          neo1.set(-1);
          neo2.set(1);
      }
  }
}