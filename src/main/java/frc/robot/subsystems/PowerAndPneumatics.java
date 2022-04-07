// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

public class PowerAndPneumatics extends SubsystemBase {
  private PneumaticHub pneumaticHub = new PneumaticHub(Ports.kPnuematicsHubCANID);
  private PowerDistribution pdp = new PowerDistribution(Ports.kPDHCANID, ModuleType.kRev);
  private boolean autoMode;
  private Timer timer = new Timer();
  /** Creates a new Power. */
  public PowerAndPneumatics() {
    timer.start();
    //compressorOn();
    compressorOff();
  }
  public void compressorOn(){
    pneumaticHub.enableCompressorAnalog(90,115);
  }
  public void compressorOff(){
    pneumaticHub.disableCompressor();
  }
  public double getPressure(){
    return pneumaticHub.makeCompressor().getPressure();
  }
  public double getCompressorWattage(){
    return pneumaticHub.getCompressorCurrent()*pdp.getVoltage();
  }
  
  public double getVoltage(){
    return pdp.getVoltage();
  }
  public double getCurrent(int channel){
    return pdp.getCurrent(channel);
  }
  public double getWattage(){
    return pdp.getTotalPower();
  }

  public void setSwitchableChannel(boolean value){
    pdp.setSwitchableChannel(value);
  }

  public void setAutoMode(boolean value){autoMode=value;}

  @Override
  public void periodic() {
    if(autoMode==false){
      setSwitchableChannel(true);
    }
    else{
      setSwitchableChannel(Math.floor(timer.get())%2==0); //blinks the lights
    }
    SmartDashboard.putNumber("Pressure", pneumaticHub.getPressure(0));
  }
}
