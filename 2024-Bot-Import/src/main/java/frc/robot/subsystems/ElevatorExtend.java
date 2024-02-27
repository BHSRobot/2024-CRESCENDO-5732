// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.MechConstants;

public class ElevatorExtend extends ProfiledPIDSubsystem {
  private CANSparkMax elevExtendNEO;
  private RelativeEncoder m_LenEncoder;
  private ElevatorFeedforward m_feedforward;
  /** Creates a new ElevatorExtend. */
  public ElevatorExtend() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            0.04,
            0,
            .03,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(.75, .15)));
    elevExtendNEO = new CANSparkMax(15, MotorType.kBrushless);
    elevExtendNEO.setIdleMode(IdleMode.kBrake);

    m_feedforward = new ElevatorFeedforward(
      0,
      MechConstants.kGElevExt,
      MechConstants.kVElevExt,
      MechConstants.kAElevExt);

    m_LenEncoder = elevExtendNEO.getEncoder();
    m_LenEncoder.setPositionConversionFactor(MechConstants.kElevLenConversionFactor);
    setGoal(0);
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    super.periodic();
    Logger.recordOutput("Encoder Value: ", getMeasurement());
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // TODO Auto-generated method stub
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    elevExtendNEO.setVoltage(output + feedforward);
  }

  @Override
  public double getMeasurement() {
    return m_LenEncoder.getPosition();
  }
}
