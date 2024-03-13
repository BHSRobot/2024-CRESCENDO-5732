// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Elevator.ElevatorExtend;
import frc.robot.subsystems.Shooter_Box.ShooterBoxPivot;

/** Add your docs here. */
public class ScoringPositions {
    public Command scoreAmpPos(ElevatorExtend elev, ShooterBoxPivot shoot) {
        return new ParallelCommandGroup(
            Commands.runOnce(() -> elev.setGoal(.78), elev),
            Commands.runOnce(() -> shoot.setGoal(101.5), shoot)
        );
    }

    public Command zero(ElevatorExtend elev, ShooterBoxPivot shoot) {
        return new ParallelCommandGroup(
            Commands.runOnce(() -> elev.setGoal(.78), elev),
            Commands.runOnce(() -> shoot.setGoal(101.5), shoot)
        );
    }
}
