// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  
  private static Elevator instance;

  private TalonFX follower;
  private TalonFX leader;

  public static Elevator getInstance(){
    if(instance == null){
      instance = new Elevator();
    }
    return instance;
  }

  public enum ElevatorState {
    L1(0),
    L2(1),
    L3(2),
    L4(3);
    private double encoderPosition;
    private ElevatorState(double encoderPosition){
      this.encoderPosition = encoderPosition;
    }
    public double getEncoderPosition(){
      return encoderPosition;
    }
  }


  public Elevator() {
    leader = new TalonFX(Constants.HardwarePorts.elevatorLeaderId);
    follower = new TalonFX(Constants.HardwarePorts.elevatorFollowerId);
    follower.setControl(new Follower(Constants.HardwarePorts.elevatorLeaderId, false));

  }

  private void configMotor(TalonFX motor, boolean inverted, NeutralModeValue neutralMode){
    
  }

  public double getPosition(){
    return leader.getPosition().getValueAsDouble();
  }

  public void stop(){
    leader.set(0);
  }

  public double getVelocity(){
    return leader.getVelocity().getValueAsDouble();
  }

  public void setVoltage(double voltage){
    leader.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
