// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry camMode = table.getEntry("camMode");
  NetworkTableEntry pipeLine = table.getEntry("pipeline");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  boolean hasTarget(){return tv.getNumber(0).intValue()==1;}
  double getX() {return tx.getDouble(0.0);}
  double getY() {return ty.getDouble(0.0);}
  double getArea() {return ta.getDouble(0.0);}

  void setDrivermode(boolean set){camMode.setNumber(set?1:0);}
  void setPipeline(int pipe){pipeLine.setNumber(pipe);}

  /** Creates a new Vision. */
  public Vision() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
