// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;
import frc.robot.Constants;
import frc.robot.Constants.MechConstants;

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
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorPivot extends ProfiledPIDSubsystem {
  private CANSparkMax m_ElevAngle;

  private RelativeEncoder m_EncAngEncoder;

  private ArmFeedforward m_feedforward;

  /** Creates a new Elevator. */
  public ElevatorPivot() {
    super(
      new ProfiledPIDController(
        MechConstants.kPElevAngle,
        MechConstants.kIElevAngle,
        MechConstants.kIElevAngle,
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

    //Absolute
    m_EncAngEncoder = m_ElevAngle.getEncoder();
    m_EncAngEncoder.setPosition(0);
    setGoal(0);
    m_ElevAngle.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
    Logger.recordOutput("Elevator Pivot Angle", getMeasurement());
  }

  @Override
  protected void useOutput(double output, State setpoint) {
    // TODO Auto-generated method stub
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    m_ElevAngle.setVoltage(output + feedforward);
  }

  @Override
  protected double getMeasurement() {
    // TODO Auto-generated method stub
    return m_EncAngEncoder.getPosition() * 20;
  }
}
