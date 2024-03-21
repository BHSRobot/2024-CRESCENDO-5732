// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter_Box.ShooterBoxPivot;
import frc.utils.Constants;
import frc.utils.Constants.MechConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorPivot extends ProfiledPIDSubsystem {
  private CANSparkMax m_ElevAngle;

  private RelativeEncoder m_EncAngEncoder;

  private ArmFeedforward m_feedforward;

  private static double staticGoal;

  /** Creates a new Elevator. */
  public ElevatorPivot() {
    super(
      new ProfiledPIDController(
        MechConstants.kPElevAngle,
        MechConstants.kIElevAngle,
        MechConstants.kDElevAngle,
          new TrapezoidProfile.Constraints
            (MechConstants.kElevAngleMaxVelocity, 
            MechConstants.kElevAngleMaxAcceleration)
        ), 
        0);
    m_ElevAngle = new CANSparkMax(MechConstants.kElevPivID, MotorType.kBrushless);
    m_feedforward = new ArmFeedforward(
      0,
      MechConstants.kGElevAng,
      MechConstants.kVElevAng,
      MechConstants.kAElevAng);

    m_EncAngEncoder = m_ElevAngle.getEncoder();
    m_EncAngEncoder.setPositionConversionFactor(MechConstants.kElevAngleConversionFactor);
    m_EncAngEncoder.setPosition(0);
    setGoal(0);
    m_ElevAngle.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
    Logger.recordOutput("Elevator Pivot Angle", getMeasurement());
    SmartDashboard.putNumber("Elevator Pivot Angle", getMeasurement());

    if (DriverStation.isDisabled())
      setGoal(staticGoal);
  }

  @Override
  protected void useOutput(double output, State setpoint) {
    // TODO Auto-generated method stub
    m_ElevAngle.set(output);
  }

  @Override
  public double getMeasurement() {
    return m_EncAngEncoder.getPosition();
  }

  public void setPermGoal(double goal) {
    staticGoal = goal;
  }
}
