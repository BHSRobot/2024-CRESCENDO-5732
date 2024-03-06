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
import frc.robot.subsystems.Elevator.ElevatorExtend.ElevExtState;
import frc.robot.subsystems.Shooter_Box.ShooterBox;
import frc.robot.subsystems.Shooter_Box.ShooterBoxPivot;
import frc.robot.subsystems.Shooter_Box.ShooterBox.ShooterState;
import frc.robot.subsystems.Shooter_Box.ShooterBoxPivot.ShootPivState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  //Elevator stuff
  // private final ElevatorExtend m_ElevatorExtend = new ElevatorExtend();
  // private final ElevatorPivot m_ElevatorPivot = new ElevatorPivot();
  //Intake/Index
  private final Intake m_Intake = new Intake();
  private final Indexer m_Indexer = new Indexer();
  //Shooter box stuff
  private final ShooterBox m_ShooterBox = new ShooterBox();
  private final ShooterBoxPivot m_ShooterBoxPivot = new ShooterBoxPivot();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_OpController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  private final LoggedDashboardChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser());
    SmartDashboard.putData("Auto Mode", AutoBuilder.buildAutoChooser());

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
                true, true),
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

       
      // m_OpController.y().
      //  onTrue(
      //    Commands.runOnce(
      //      () -> {
      //        m_ShooterBoxPivot.setGoal(.6);
      //        m_ShooterBoxPivot.enable();
      //      },
      //      m_ShooterBoxPivot
      //    )
      //  );

      //  m_OpController.x().
      //  onTrue(
      //    Commands.runOnce(
      //      () -> {
      //        m_ShooterBoxPivot.setGoal(MechConstants.kWristAngleOffest);
      //        m_ShooterBoxPivot.enable();
      //      },
      //      m_ShooterBoxPivot
      //    )
      //  );
       
      
    
    //Driver Intake
    m_driverController.rightTrigger().
      onTrue(m_Intake.intakeCommand().alongWith(m_Indexer.forwardCommand()))
      .onFalse(m_Intake.disabledCommand().alongWith(m_Indexer.disabledCommand()));
    
    m_driverController.leftTrigger().
      onTrue(m_Intake.ejectCommand().alongWith(m_Indexer.backCommand()))
      .onFalse(m_Intake.disabledCommand().alongWith(m_Indexer.disabledCommand())); 

    //Shooter Box Front Shooter(s)
    m_OpController.rightTrigger().
      onTrue(m_ShooterBox.enabledCommand())
      .onFalse(m_ShooterBox.disabledCommand());    
  }

  public Command shootThenBackUp() {
    return new RunCommand(
      () -> m_ShooterBox.setShooterSpeed(1),
      m_ShooterBox).
    withTimeout(2.5).
    andThen(
      new ParallelCommandGroup(
        new RunCommand(() -> m_Indexer.setShooterSpeed(.25), m_Indexer),
        new RunCommand(() -> m_ShooterBox.setShooterSpeed(1), m_ShooterBox)
      ).
      withTimeout(3)
    ).andThen(
      shootThenTaxiTrajectoryCommand()
    );
  }

  public Command shootThenTaxiTrajectoryCommand() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("shootThenTaxi.path");

    PathConstraints constraints = new PathConstraints(
      .85,
      .05,
      2.35,
      .55);
    Command pathFindCommand = AutoBuilder.pathfindThenFollowPath(path, constraints);

    return pathFindCommand;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new RunCommand(
      () -> m_robotDrive.drive
      (.25, 0, 0, false, false),
      m_robotDrive
    )
    .withTimeout(1.5);
  }          
}
