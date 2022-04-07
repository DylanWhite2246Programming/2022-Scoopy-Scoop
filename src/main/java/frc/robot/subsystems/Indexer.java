// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorControllerValues;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ScoopConstants;

public class Indexer extends SubsystemBase {
  private WPI_VictorSPX intake = new WPI_VictorSPX(Ports.kIntakeCANID);
  private WPI_VictorSPX belt = new WPI_VictorSPX(Ports.kBeltCANID);
  /** Creates a new Indexer. */
  public Indexer() {
    intake.setInverted(ScoopConstants.kIntakeInversed);
    belt.setInverted(ScoopConstants.kRollerInversed);
  }

  public void beltForward(){belt.set(1);}
  public void beltReverse(){belt.set(-1);}
  public void beltSTOP(){belt.stopMotor();}

  public void intakeForward(){intake.set(MotorControllerValues.kIntakeValue);}
  public void intakeReverse(){intake.set(-MotorControllerValues.kIntakeValue);}
  public void intakeSTOP(){intake.stopMotor();}

  public void VOMIT(){
    beltReverse();
    intakeReverse();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
