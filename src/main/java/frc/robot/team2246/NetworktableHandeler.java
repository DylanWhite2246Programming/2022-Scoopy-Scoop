// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.team2246;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LifterConstants;

public class NetworktableHandeler extends SubsystemBase {
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table;
  private NetworkTableEntry 
    overideAutoPipe,
    pipeIndex,
    liftSetpoint;

  /** Creates a new NetworkTableHandeler. */
  public NetworktableHandeler() {
    table = inst.getTable("test");
    //overideAutoPipe = new NetworkTableEntry(inst, 1);
    //pipeIndex = new NetworkTableEntry(inst, 1);
    //liftSetpoint = new NetworkTableEntry(inst, 1);
    liftSetpoint = Shuffleboard.getTab("Lifter")
      .add("Setpoint", LifterConstants.kOffSet)
      .getEntry();
  }
  public double getLiftSetpoint(){
    return liftSetpoint.getDouble(LifterConstants.kOffSet);
  }
  public boolean getOverideAutoPipe(){
    return overideAutoPipe.getBoolean(false);
  }
  public int getManualPipe(){
    return pipeIndex.getNumber(0).intValue();
  }

  @Override
  public void periodic() {
    Shuffleboard.update();
    //System.out.println(getLiftSetpoint());
    // This method will be called once per scheduler run
  }
}
