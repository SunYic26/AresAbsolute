// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZeroElevator extends Command {
  private Elevator s_Elevator;

  public ZeroElevator(){
    s_Elevator = Elevator.getInstance();
    addRequirements(s_Elevator);
  }

  @Override
  public void initialize() {
    s_Elevator.setSpeed(-0.05);
  }


  @Override
  public void end(boolean interrupted) {
    s_Elevator.stop();
    if(!interrupted){
      s_Elevator.zeroPosition();
      System.out.println("Zero Elevator Ended");
    }
    else{
      System.out.println("ZeroElevatorInterrupted");}
    
  }

  @Override
  public boolean isFinished() {
    return s_Elevator.getStatorCurrent() > Constants.elevatorCurrentThreshold;
  }
}
