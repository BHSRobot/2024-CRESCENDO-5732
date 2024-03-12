// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Shooter_Box.Indexer;
import frc.robot.subsystems.Shooter_Box.ShooterBox;

/** Add your docs here. */
public class Autos {
    private final LoggedDashboardChooser<Command> autoChooser;
 
    public Autos() { 
        autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser());
    }

    public Command shootThenBackUp(ShooterBox shootBox, Indexer index) {
        return new RunCommand(
            () -> shootBox.setShooterSpeed(1),
            shootBox).
        withTimeout(2.5).
        andThen(
            new ParallelCommandGroup(
            new RunCommand(() -> shootBox.setShooterSpeed(-1), shootBox),
            new RunCommand(() -> index.setIndexerSpeed(.25), index)
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

}
