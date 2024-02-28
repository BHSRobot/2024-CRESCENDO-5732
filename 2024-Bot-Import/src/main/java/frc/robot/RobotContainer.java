// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MechConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator.ElevatorExtend;
import frc.robot.subsystems.Elevator.ElevatorPivot;
import frc.robot.subsystems.Shooter_Box.ShooterBox;
import frc.robot.subsystems.Shooter_Box.ShooterBoxPivot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ElevatorExtend m_ElevatorExtend = new ElevatorExtend();
  //private final ElevatorPivot m_ElevatorPivot = new ElevatorPivot();
  private final Intake m_Intake = new Intake();
  private final Indexer m_Indexer = new Indexer();
  //private final ShooterBox m_ShooterBox = new ShooterBox();
  //private final ShooterBoxPivot m_ShooterBoxPivot = new ShooterBoxPivot();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  //private final LoggedDashboardChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    //autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser());
    //SmartDashboard.putData("Auto Mode", AutoBuilder.buildAutoChooser());

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, false),
            m_robotDrive));
  
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
   m_driverController.y().
      whileTrue(
        new InstantCommand(() -> m_robotDrive.setX(), m_robotDrive));

    //Position 1
    /*m_driverController.x()
    .onTrue(
        new ParallelCommandGroup(
          Commands.runOnce(
          () -> {
            m_ElevatorExtend.setGoal(1.0);
            m_ElevatorExtend.enable();
          }
          , m_ElevatorExtend),
          Commands.runOnce(
          () -> {
            m_ElevatorPivot.setGoal(1.0);
            m_ElevatorPivot.enable();
          }
          , m_ElevatorPivot),
          Commands.runOnce(
          () -> {
            m_ShooterBoxPivot.setGoal(1.0);
            m_ShooterBoxPivot.enable();
          }
          , m_ShooterBoxPivot)
        ));

    //Position 2
    m_driverController.a()
      .onTrue(
        Commands.runOnce(
          () -> {
            m_ElevatorExtend.setGoal(0);
            m_ElevatorExtend.enable();
          }
          , m_ElevatorExtend));*/

    //Position 3
    m_driverController.b()
      .onTrue(
        Commands.runOnce(
          () -> {
            m_ElevatorExtend.setGoal(null);
            m_ElevatorExtend.enable();
          }
          , m_ElevatorExtend));

    //Elevator manual move
    m_driverController.x().
          onTrue(
            new RunCommand(
              () -> m_ElevatorExtend.setManualSpeed(.5),
              m_ElevatorExtend)).
          onFalse(
            new RunCommand(
              ()-> m_ElevatorExtend.setManualSpeed(0),
              m_ElevatorExtend));

    m_driverController.rightTrigger().
      onTrue(m_Intake.intakeCommand())
      .onFalse(m_Intake.disabledCommand());
    
    m_driverController.leftTrigger().
      onTrue(m_Intake.ejectCommand())
      .onTrue(m_Intake.disabledCommand()); 
      
    /*m_driverController.rightBumper().
      whileTrue(m_ShooterBox.enabledCommand())
      .whileFalse(m_Intake.disabledCommand());

    m_driverController.leftBumper().
      whileTrue(m_Indexer.forwardCommand())
      .whileFalse(m_Indexer.disabledCommand());*/

    
    
    //SmartDashboard.putData("Example Auto", new PathPlannerAuto("New Auto"));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return /*autoChooser.get()*/ null;
  }
}
