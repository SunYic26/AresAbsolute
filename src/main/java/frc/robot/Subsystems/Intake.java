// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  
  private TalonFX leader;
  private TalonFX follower;
  private TalonFX roller;
  
  private static Intake instance;

  public static Intake getInstance(){
    if(instance == null){
      instance = new Intake();
    }
    return instance;
  }

  public Intake() {
    leader = new TalonFX(Constants.HardwarePorts.intakeLeaderID, "mechbus");
    follower = new TalonFX(Constants.HardwarePorts.intakeFollowerID, "mechbus");
    roller = new TalonFX(Constants.HardwarePorts.intakeRollerID, "mechbus");
    follower.setControl(new Follower(Constants.HardwarePorts.intakeLeaderID, true));

    configPivot(leader, NeutralModeValue.Brake, InvertedValue.Clockwise_Positive);
    configPivot(follower, NeutralModeValue.Brake, InvertedValue.CounterClockwise_Positive);
    configRoller(roller, NeutralModeValue.Brake, InvertedValue.Clockwise_Positive);
  }

  private void configPivot(TalonFX motor, NeutralModeValue neutralMode, InvertedValue direction){
    // motor.setNeutralMode(neutralMode);
    TalonFXConfiguration config = new TalonFXConfiguration();
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    config.MotorOutput.Inverted = direction;
    currentLimitsConfigs.SupplyCurrentLimit = Constants.CurrentLimits.intakeContinuousCurrentLimit;
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;
    currentLimitsConfigs.StatorCurrentLimit = Constants.CurrentLimits.intakePeakCurrentLimit;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = neutralMode;

    config.CurrentLimits = currentLimitsConfigs;

    Slot0Configs position = new Slot0Configs();
    position.kP = 1.4;
    position.kI = 0;


    motor.getConfigurator().apply(config);
    motor.getConfigurator().apply(position);
  }


  private void configRoller(TalonFX motor, NeutralModeValue neutralMode, InvertedValue direction){
    // motor.setNeutralMode(neutralMode);
    TalonFXConfiguration config = new TalonFXConfiguration();
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    config.MotorOutput.Inverted = direction;
    currentLimitsConfigs.SupplyCurrentLimit = Constants.CurrentLimits.intakeContinuousCurrentLimit;
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;
    currentLimitsConfigs.StatorCurrentLimit = Constants.CurrentLimits.intakePeakCurrentLimit;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = neutralMode;

    config.CurrentLimits = currentLimitsConfigs;

    Slot0Configs position = new Slot0Configs();
    position.kP = 1.7;

    motor.getConfigurator().apply(config);
    motor.getConfigurator().apply(position);
  }

  public enum PivotState{
    UP(0), //arbitrary numbers for now
    HOLD(1.433),
    DOWN(5.4);

    private double position;
    private PivotState(double position){
      this.position = position;
    }
    public double getPosition(){
      return position; 
    }
  }

  public enum RollerState{
    INTAKE(0.5),
    OUTTAKE(-0.5),
    OFF(0);
    private double motorSpeed;
    private RollerState(double motorSpeed){
      this.motorSpeed = motorSpeed;
    }
    public double getRollerSpeed(){
      return motorSpeed;
    }
  }

  public void brakeRoller(){
    roller.setControl(new PositionVoltage(roller.getPosition().getValueAsDouble()));
  }

  

  // public void testUnbrake(){
  //   roller.setControl(new VoltageOut(0.3));
  // }

  public void stopPivot(){
    leader.set(0);
  }

  public double getPosition(){
    return leader.getPosition().getValueAsDouble();
  }

  public double getPivotCurrent(){
    return leader.getStatorCurrent().getValueAsDouble();
  }

  public void brakePivot(){
    leader.setControl(new PositionVoltage(leader.getPosition().getValueAsDouble()));
  }

  public void setPivotSpeed(double speed){
    leader.set(speed);
  }

  public void setPivotPosition(PivotState state){
    leader.setControl(new PositionVoltage(state.getPosition()));
  }

  public double getRollerCurrent(){
    return roller.getStatorCurrent().getValueAsDouble();
  }
  
  public void resetPivotPosition(){
    leader.setPosition(0);
  }

  public void setPivotVoltage(double voltage){
    leader.setControl(new VoltageOut(voltage));
  }

  public void setRollerSpeed(double speed){
    roller.set(speed);
  }

  public void setRollerVoltage(double voltage){
    roller.setControl(new VoltageOut(voltage));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("intake pivot position", getPosition());
  }
}
