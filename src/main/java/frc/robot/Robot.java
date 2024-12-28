// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.AccelerationIntegrator;
import frc.robot.RobotState.RobotState;
// import frc.robot.Subsystems.CommandSwerveDrivetrain.DriveControlSystems;
import frc.robot.Subsystems.CommandSwerveDrivetrain.Drivetrain;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.Autos;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  SendableChooser<AutoCommand> firstAuto = new SendableChooser<AutoCommand>();
  SendableChooser<AutoCommand> secondAuto = new SendableChooser<AutoCommand>();
  SendableChooser<AutoCommand> thirdAuto = new SendableChooser<AutoCommand>();
  SendableChooser<AutoCommand> fourthAuto = new SendableChooser<AutoCommand>();
  SendableChooser<AutoCommand> fifthAuto = new SendableChooser<AutoCommand>();

  private AutoCommand firstSavedChoice;
  private AutoCommand secondSavedChoice;
  private AutoCommand thirdSavedChoice;
  private AutoCommand fourthSavedChoice;
  private AutoCommand fifthSavedChoice;

  private RobotContainer m_robotContainer;

  private final Vision vision;

  private Drivetrain drivetrain;
  private RobotState robotState;
 
    public Robot() { //need stuff for logger at some point
      drivetrain = Drivetrain.getInstance();
      robotState = RobotState.getInstance();
      vision = Vision.getInstance();
    }

  @Override
  public void robotInit() {
    firstAuto.addOption(AutoCommand.Test1().name, AutoCommand.Test1());
    firstAuto.addOption(AutoCommand.Test2().name, AutoCommand.Test2());
    AutoCommand.loadAutos();
    SmartDashboard.putData("first auto", firstAuto);

    if(isReal()){
      Logger.addDataReceiver(new WPILOGWriter()); // should be savig to usb
      Logger.addDataReceiver(new NT4Publisher());
    }
    else {
      Logger.addDataReceiver(new NT4Publisher());
    }
    Logger.start();

    //start the logger here
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    if(firstAuto.getSelected() != firstSavedChoice){ //Note: might be able to use the onchange() method in sendable chooser
      firstSavedChoice = firstAuto.getSelected();
      updateSecondAuto();
    }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  private void updateSecondAuto(){
    if(secondAuto != null){
      secondAuto.close();
    }
    secondAuto = new SendableChooser<AutoCommand>();
    firstSavedChoice = firstAuto.getSelected();
    AutoCommand.clearContinuations();
    AutoCommand.fillAutosList(firstSavedChoice);
      for(int i = 0; i < firstAuto.getSelected().getPotentialContinuations().size(); i++){
        secondAuto.addOption(firstAuto.getSelected().name, firstAuto.getSelected().getPotentialContinuations().get(i));
      }
      SmartDashboard.putData("second auto", secondAuto);
  }
}

