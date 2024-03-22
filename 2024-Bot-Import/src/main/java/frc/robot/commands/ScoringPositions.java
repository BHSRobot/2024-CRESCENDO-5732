// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Elevator.ElevatorExtend;
import frc.robot.subsystems.Elevator.ElevatorPivot;
import frc.robot.subsystems.Shooter_Box.ShooterBoxPivot;

/** Add your docs here. */
public class ScoringPositions {
    public Command scoreAmpPos(ElevatorExtend elev, ShooterBoxPivot shoot) {
        return new ParallelCommandGroup(
            Commands.runOnce(
                () ->  {
                    elev.setGoal(.78);
                    elev.enable();
                },
                elev),
            Commands.runOnce(
                () ->  {
                    shoot.setGoal(101.5);
                    shoot.enable();
                },
                shoot)
        );
    }

    public Command zeroAmp(ElevatorExtend elev, ShooterBoxPivot shoot, ElevatorPivot elevPiv) {
        return new ParallelCommandGroup(
            Commands.runOnce(
            () ->  {
                elev.setGoal(0);
                elev.enable();
            }, elev),
            Commands.runOnce(
            () ->  {
                shoot.setGoal(0.0);
                shoot.enable();
            }, shoot)
            );
    }

    public Command climb(ElevatorExtend elev, ShooterBoxPivot shoot, ElevatorPivot elevPiv) {
        return new ParallelCommandGroup(
            Commands.runOnce(
            () ->  {
                elev.setGoal(1);
                elev.enable();
            }, elev),
            Commands.runOnce(
            () ->  {
                shoot.setGoal(90);
                shoot.enable();
            }, shoot))
        .withTimeout(1.2).
        andThen(Commands.runOnce(
        () -> {
            elevPiv.setGoal(30);
            elevPiv.setPermGoal(30);
            elevPiv.enable();
        },
        elevPiv));
    }

    public Command climbZero(ElevatorExtend elev, ShooterBoxPivot shoot, ElevatorPivot elevPiv) {
        return Commands.runOnce(
            () -> {
                elevPiv.setGoal(0);
                elevPiv.setPermGoal(0);
                elevPiv.enable();
            }, elevPiv).withTimeout(1.2)
            .andThen(
                new ParallelCommandGroup(
                    Commands.runOnce(
                    () ->  {
                        elev.setGoal(0);
                        elev.enable();
                    }, elev),
                    Commands.runOnce(
                    () ->  {
                        shoot.setGoal(0);
                        shoot.enable();
                    }, shoot)
                ));

    }

    public Command defaultSpeaker(ElevatorPivot elevPiv) {
        return Commands.runOnce(
        () -> {
            elevPiv.setGoal(7.5);
            elevPiv.setPermGoal(7.5);
            elevPiv.enable();
        },
        elevPiv);
    }

    public Command defaultSpeakerZero(ElevatorPivot elevPiv) {
        return Commands.runOnce(
        () -> {
            elevPiv.setGoal(0);
            elevPiv.setPermGoal(0);
            elevPiv.enable();
        },
        elevPiv)
        .withTimeout(1)
        .andThen(Commands.runOnce(() -> elevPiv.disable(), elevPiv));
    }
}
