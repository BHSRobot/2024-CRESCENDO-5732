// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AimWithLimelight;
import frc.robot.commands.Autos;
import frc.robot.commands.ScoringPositions;
import frc.robot.subsystems.Elevator.ElevatorExtend;
import frc.robot.subsystems.Elevator.ElevatorPivot;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter_Box.Indexer;
import frc.robot.subsystems.Shooter_Box.ShooterBox;
import frc.robot.subsystems.Shooter_Box.ShooterBoxPivot;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import frc.utils.LimeHelp;
import frc.utils.Constants.AutoConstants;
import frc.utils.Constants.DriveConstants;
import frc.utils.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final static DriveSubsystem m_robotDrive = new DriveSubsystem();
  //Elevator stuff
  private final ElevatorExtend m_ElevatorExtend = new ElevatorExtend();
  private final ElevatorPivot m_ElevatorPivot = new ElevatorPivot();
  //Intake/Index
  public final static Intake m_Intake = new Intake();
  public final static Indexer m_Indexer = new Indexer();
  //Shooter box stuff
  public final static ShooterBox m_ShooterBox = new ShooterBox();
  public final static ShooterBoxPivot m_ShooterBoxPivot = new ShooterBoxPivot();

  private final LoggedDashboardChooser<Command> autoChooser;


  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_OpController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  public RobotContainer() {

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
                true, false),
            m_robotDrive));

    configureNamedCommands();

    autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser());
  }

  public void configureNamedCommands() {
    NamedCommands.registerCommand("Auto Amp-Score",
      new RunCommand(() -> m_ShooterBox.setShooterSpeed(-1), m_ShooterBox)
      .alongWith(new ScoringPositions().scoreAmpPos(m_ElevatorExtend, m_ShooterBoxPivot)
      .andThen(new RunCommand(() -> m_Indexer.setIndexerSpeed(.75))))
    );
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
  //X-Stance Defence Position
    m_driverController.y().
      whileTrue(new InstantCommand(() -> m_robotDrive.setX(), m_robotDrive));
    //Amp Score
    m_OpController.povRight().
      onTrue(new ScoringPositions().scoreAmpPos(m_ElevatorExtend, m_ShooterBoxPivot));
    //Default Position
    m_OpController.povDown().
      onTrue(new ScoringPositions().zero(m_ElevatorExtend, m_ShooterBoxPivot));
    //Limelight Auto-Aim
    m_OpController.x().
      onTrue(Commands.runOnce(
        () -> m_ShooterBoxPivot.setGoal(new LimeHelp().getTY()),
        m_ShooterBoxPivot)
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
    
    //Driver Aligns to AprilTag
    m_driverController.a().
      onTrue(new ConditionalCommand(
        new RunCommand(() -> m_robotDrive.drive(0, 0, new LimeHelp().aimRobotRot(), false, false), m_robotDrive),
        new RunCommand(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive),
        () ->  new LimeHelp().getTX() <= 0
        )
      );

    //Drive Aligns to AprilTag (the dumb way)
    /*
    m_driverController.a().
      onTrue(new AimWithLimelight(m_robotDrive)); 
     */
  }

  public void setupDriverTab() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    driverTab.addDouble("Time Remaining", () -> {
        return Timer.getMatchTime();
      }
    );
    driverTab.addString("Event Name",  () -> { 
        return DriverStation.getEventName();
      }
    );
    driverTab.addString("Alliance Color",  () -> { 
        return DriverStation.getAlliance().toString();
      }
    );
    
    CameraServer.startAutomaticCapture();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }          
}
