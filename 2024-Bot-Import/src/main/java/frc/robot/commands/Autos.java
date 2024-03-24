// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class Autos { 
    public Autos() {
    }

    //Speaker Score then Taxi Auto

    public Command shootSpeaker() {
        Command aimShooter = Commands.runOnce(
            () -> {
                RobotContainer.m_ElevatorPivot.setGoal(265);
                RobotContainer.m_ElevatorPivot.enable();
            },
            RobotContainer.m_ElevatorPivot
            );
        Command lowerShooter = Commands.runOnce(
            () -> {
                RobotContainer.m_ElevatorPivot.disable();
            },
            RobotContainer.m_ElevatorPivot
            );
        Command revShooterBox = new RunCommand(
            () -> RobotContainer.m_ShooterBox.setShooterSpeed(-1),
            RobotContainer.m_ShooterBox);
        Command runIndexerUp = new RunCommand(
            () -> RobotContainer.m_Indexer.setIndexerSpeed(.5),
            RobotContainer.m_Indexer);
        return new ParallelCommandGroup(
            revShooterBox,
            aimShooter,
            new WaitCommand(4.5).andThen(runIndexerUp)
        ).withTimeout(5)
        .andThen(lowerShooter);
    }
}
