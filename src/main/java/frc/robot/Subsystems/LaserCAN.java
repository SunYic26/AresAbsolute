// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RangingMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//Should be integrated into intake subsystem, but we don't have a mechanism for that yet so it's being kept separate for now
public class LaserCAN extends SubsystemBase {
  
  private double laserThreshold = 15; //depends on lasercan placement
  private LaserCan laser;

  public LaserCAN() {
    laser = new LaserCan(Constants.HardwarePorts.laserID);
    configLaser();
  }

  private void configLaser(){
    try{
      laser.setRangingMode(RangingMode.SHORT);
      laser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
      laser.setRegionOfInterest(new LaserCan.RegionOfInterest(0, 0, 0, 0)); 
    }
    catch(ConfigurationFailedException e){
      SmartDashboard.putBoolean("laser working", false);
    }
  }

  public double getLaserMeasurement(){
    return laser.getMeasurement().distance_mm;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
