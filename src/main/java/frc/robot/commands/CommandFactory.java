// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Intake.PivotState;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Intake.RollerState;

/** Add your docs here. */
public class CommandFactory {

    public static Command Intake(){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SetIntakePivot(PivotState.DOWN),
                new InstantCommand(()-> Intake.getInstance().setRollerSpeed(RollerState.INTAKE.getRollerSpeed()))
                ),
            Commands.waitSeconds(0.1),
            new ParallelCommandGroup(
                new SetIntakePivot(PivotState.UP),
                new InstantCommand(()-> Intake.getInstance().setRollerSpeed(RollerState.OFF.getRollerSpeed()))
            )
        );
    }

    public static Command Outtake() {
        return new InstantCommand(()->Intake.getInstance().setRollerSpeed(RollerState.OUTTAKE.getRollerSpeed()));
    }
}
