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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator.ElevatorExtend;
import frc.robot.subsystems.Elevator.ElevatorPivot;
import frc.robot.subsystems.Elevator.ElevatorExtend.ElevExtState;
import frc.robot.subsystems.Shooter_Box.Indexer;
import frc.robot.subsystems.Shooter_Box.ShooterBox;
import frc.robot.subsystems.Shooter_Box.ShooterBoxPivot;
import frc.robot.subsystems.Shooter_Box.ShooterBox.ShooterState;
import frc.robot.subsystems.Shooter_Box.ShooterBoxPivot.ShootPivState;
import frc.utils.Constants.AutoConstants;
import frc.utils.Constants.DriveConstants;
import frc.utils.Constants.MechConstants;
import frc.utils.Constants.OIConstants;
import frc.robot.subsystems.Intake;
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

import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

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
  private final HashMap<String, Supplier<Command>> autoList;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {
    autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser());
    autoList = new HashMap<String, Supplier<Command>>();
    SmartDashboard.putData("Auto Mode", AutoBuilder.buildAutoChooser());

    // Configure the button bindings
    configureButtonBindings();
    
    setupDriverTab();


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

      //Note to self, elevator goal should be .70 and then .05
      //Shooter pivot should be 100 then .15
      m_OpController.y().
       onTrue(
         Commands.runOnce(
           () -> {
             m_ShooterBoxPivot.setGoal(90);
             m_ShooterBoxPivot.enable();
             System.out.println("PID ON");
           },
           m_ShooterBoxPivot
         )
       );

       m_OpController.x().
       onTrue(
         Commands.runOnce(
           () -> {
             m_ShooterBoxPivot.setGoal(MechConstants.kWristAngleOffest);
             m_ShooterBoxPivot.enable();
             System.out.println("PID OFF");
           },
           m_ShooterBoxPivot
         )
       );
       
      
    
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

  public void setupDriverTab() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");

    driverTab.addDouble("Time Remaining", () -> { return (int) Timer.getMatchTime();});
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    m_robotDrive.resetOdometry(m_robotDrive.getPose());
    return autoChooser.get();
  }          
}
