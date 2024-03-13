// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter_Box.Indexer;
import frc.robot.subsystems.Shooter_Box.ShooterBox;

/** Add your docs here. */
public class Autos {
    public final LoggedDashboardChooser<Command> autoChooser;
 
    public Autos() { 
        autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser());
        SmartDashboard.putData("Auto Mode", AutoBuilder.buildAutoChooser());
    }
    //Speaker Score then Taxi Auto
    public Command shootThenTaxiTrajectoryCommand() {
        PathPlannerPath path = PathPlannerPath.fromPathFile("shootThenTaxi.path"); //First we initialize our path from a file located within our project

        PathConstraints constraints = new PathConstraints( //Then we declare the constraints of followng said path
        .85,
        .05,
        2.35,
        .55);
        Command pathFindCommand = AutoBuilder.pathfindThenFollowPath(path, constraints); //Afterwards, we pass the path into AutoBuilder to build the auto, and return it as a new command

        return pathFindCommand;
    }

    public Command shootThenBackUp() {
        Command shootBackUp = new RunCommand(
            () -> RobotContainer.m_ShooterBox.setShooterSpeed(1),
            RobotContainer.m_ShooterBox).
        withTimeout(2.5).
        andThen(
            new ParallelCommandGroup(
            new RunCommand(() -> RobotContainer.m_ShooterBox.setShooterSpeed(-1), RobotContainer.m_ShooterBox),
            new RunCommand(() -> RobotContainer.m_Indexer.setIndexerSpeed(.25), RobotContainer.m_Indexer)
            ).
            withTimeout(3)
        ).andThen(
            shootThenTaxiTrajectoryCommand()
        );
        autoChooser.addOption("Score Speaker then Taxi", shootBackUp);
        return shootBackUp;
    }
}
