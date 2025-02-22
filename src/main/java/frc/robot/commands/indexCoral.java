package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystems.EndEffector;
import frc.robot.Subsystems.EndEffector.OuttakeState;

public class indexCoral extends Command {
  EndEffector s_EndEffector;
  Timer timer = new Timer();
  boolean finished = false;

  public indexCoral() {
    s_EndEffector = EndEffector.getInstance();
    addRequirements(s_EndEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    s_EndEffector.setOuttakeSpeed(0.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.hasElapsed(0.1)) {
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_EndEffector.setOuttakeSpeed(0.2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}