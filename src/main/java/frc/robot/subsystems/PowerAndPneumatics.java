// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

public class PowerAndPneumatics extends SubsystemBase {
  private PneumaticHub pneumaticHub = new PneumaticHub(Ports.kPnuematicsHubCANID);
  private PowerDistribution pdp = new PowerDistribution(Ports.kPDHCANID, ModuleType.kRev);
  //private boolean autoMode;
  private Timer timer = new Timer();

  private DoubleSolenoid scoop = new DoubleSolenoid(
    Ports.kPnuematicsHubCANID, 
    PneumaticsModuleType.REVPH, 
    Ports.scoopSolenoidPort[0], 
    Ports.scoopSolenoidPort[1]
  );
  private DoubleSolenoid climber = new DoubleSolenoid(
    Ports.kPnuematicsHubCANID,
    PneumaticsModuleType.REVPH,
    Ports.climberSolenoidPort[0],
    Ports.climberSolenoidPort[1]
  );
  /** Creates a new Power. */
  public PowerAndPneumatics() {
    timer.start();
    //compressorOn();
    compressorOff();
  }

  public void climberUp(){climber.set(Value.kForward);}
  public void climbDown(){climber.set(Value.kReverse);}

  public void scoopUp(){scoop.set(Value.kForward);}
  public void scoopDown(){scoop.set(Value.kReverse);}

  public void compressorOn(){
    pneumaticHub.enableCompressorAnalog(90,112.5);
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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pressure", pneumaticHub.getPressure(0));
  }
}
