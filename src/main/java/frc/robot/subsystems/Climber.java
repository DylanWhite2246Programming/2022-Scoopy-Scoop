// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

public class Climber extends SubsystemBase {
  
  private final DoubleSolenoid backSolenoid
    = new DoubleSolenoid(
      Ports.kPnuematicsHubCANID, 
      PneumaticsModuleType.REVPH, 
      Ports.climber1Ports[0], 
      Ports.climber1Ports[1]
    );
  private final DoubleSolenoid lifterSolenoid
    = new DoubleSolenoid(
      Ports.kPnuematicsHubCANID, 
      PneumaticsModuleType.REVPH, 
      Ports.climber2Ports[0], 
      Ports.climber2Ports[1]
    );
  
  /** Creates a new Climber. */
  public Climber() {
  }

  public void extendBackSolenoid(){
    backSolenoid.set(Value.kForward);
  }
  public void retrackBackSolenoid(){
    backSolenoid.set(Value.kReverse);
  }

  public void extendLifterSolenoid(){
    lifterSolenoid.set(Value.kForward);
  }
  public void retrackLifterSolenoid(){
    lifterSolenoid.set(Value.kReverse);
  }

  public void STOP(){
    backSolenoid.set(Value.kOff);
    lifterSolenoid.set(Value.kOff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
