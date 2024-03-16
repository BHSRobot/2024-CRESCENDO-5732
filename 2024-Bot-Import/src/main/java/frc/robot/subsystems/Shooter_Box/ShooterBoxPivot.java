// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter_Box;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.subsystems.Elevator.ElevatorExtend;
import frc.robot.subsystems.Elevator.ElevatorPivot;
import frc.utils.Constants.MechConstants;

public class ShooterBoxPivot extends ProfiledPIDSubsystem {
  private CANSparkMax m_WristAngle;

  private RelativeEncoder m_WriAngEncoder;

  private ArmFeedforward m_feedforward;

  private NetworkTable table;

  /** Creates a new ShooterBoxPivot. */
  public ShooterBoxPivot() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            MechConstants.kPElevAngle,
            MechConstants.kIElevAngle,
            MechConstants.kDElevAngle,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(180, 180)));
    m_WristAngle = new CANSparkMax(MechConstants.kWristPivID, MotorType.kBrushless);
    m_feedforward = new ArmFeedforward(
      0,
      MechConstants.kGWrist,
      MechConstants.kVWrist,
      MechConstants.kAWrist);

    m_WriAngEncoder = m_WristAngle.getEncoder();
    m_WriAngEncoder.setPositionConversionFactor(MechConstants.kWristAngleConversionFactor);
    m_WristAngle.setIdleMode(IdleMode.kCoast);
    setGoal(MechConstants.kWristAngleOffest);

    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public enum ShootPivState {
    ENABLED,
    DISABLED
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
    Logger.recordOutput("Wrist Pivot Angle", getMeasurement());
    Logger.recordOutput("Wrist Pivot Setpoint", 90);
    SmartDashboard.putNumber("Wrist Pivot Angle", getMeasurement());
    Logger.recordOutput("Limelight TY", table.getEntry("ty").getDouble(0.0));

    if (DriverStation.isDisabled())
      setGoal(0.15);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    m_WristAngle.setVoltage(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_WriAngEncoder.getPosition();
  }

}
