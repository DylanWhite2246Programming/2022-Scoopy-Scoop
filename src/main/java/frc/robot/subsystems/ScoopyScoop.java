// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorControllerValues;
import frc.robot.Constants.Ports;

public class ScoopyScoop extends SubsystemBase {
  
  private final WPI_VictorSPX intake = new WPI_VictorSPX(Ports.kIntakeCANID);
  private final WPI_VictorSPX rollers = new WPI_VictorSPX(Ports.kRollerCANID);
  /** Creates a new ScoopyScoop. */
  public ScoopyScoop() {}

  public void Intake(){
    intake.set(MotorControllerValues.kIntakeValue);
  }
  public void Shoot(){
    intake.set(-MotorControllerValues.kShootValue);
  }
  public void rollerInktake(){
    rollers.set(MotorControllerValues.kRollerValues);
  }
  public void rollerShoot(){
    rollers.set(-MotorControllerValues.kRollerValues);
  }
  public void STOPAXELS(){
    intake.setNeutralMode(NeutralMode.Coast);
    rollers.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
