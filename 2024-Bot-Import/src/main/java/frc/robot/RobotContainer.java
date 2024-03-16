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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Autos;
import frc.robot.commands.ScoringPositions;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator.ElevatorExtend;
import frc.robot.subsystems.Elevator.ElevatorPivot;
import frc.robot.subsystems.Shooter_Box.Indexer;
import frc.robot.subsystems.Shooter_Box.ShooterBox;
import frc.robot.subsystems.Shooter_Box.ShooterBoxPivot;
import frc.utils.LimeHelp;
import frc.utils.Constants.AutoConstants;
import frc.utils.Constants.DriveConstants;
import frc.utils.Constants.OIConstants;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private final ElevatorExtend m_ElevatorExtend = new ElevatorExtend();
  private final ElevatorPivot m_ElevatorPivot = new ElevatorPivot();
  //Intake/Index
  public final static Intake m_Intake = new Intake();
  public final static Indexer m_Indexer = new Indexer();
  //Shooter box stuff
  public final static ShooterBox m_ShooterBox = new ShooterBox();
  public final static ShooterBoxPivot m_ShooterBoxPivot = new ShooterBoxPivot();

  private final Autos auto = new Autos();


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
    // return auto.autoChooser.get();
     // 1. Create trajectory settings
      TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
              AutoConstants.kMaxSpeedMetersPerSecond,
              AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                      .setKinematics(DriveConstants.kDriveKinematics);

      // 2. Generate trajectory
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
              new Pose2d(0, 0, new Rotation2d(0)),
              List.of(
                      new Translation2d(1, 0),
                      new Translation2d(1, -1)),
              new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
              trajectoryConfig);

      // 3. Define PID controllers for tracking trajectory
      PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
      PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
      ProfiledPIDController thetaController = new ProfiledPIDController(
              AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      // 4. Construct command to follow trajectory
      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
              trajectory,
              m_robotDrive::getPose,
              DriveConstants.kDriveKinematics,
              xController,
              yController,
              thetaController,
              m_robotDrive::setModuleStates,
              m_robotDrive);

      // 5. Add some init and wrap-up, and return everything
      return new SequentialCommandGroup(
              new InstantCommand(() -> m_robotDrive.resetOdometry(trajectory.getInitialPose())),
              swerveControllerCommand,
              new InstantCommand(() -> m_robotDrive.stopModules()));
  }          
}
