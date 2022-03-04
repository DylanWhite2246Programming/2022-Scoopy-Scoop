// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorControllerValues;
import frc.robot.Constants.Ports;

public class ScoopyScoop extends SubsystemBase {

  private final WPI_VictorSPX belt = new WPI_VictorSPX(Ports.kBeltCANID);
  private final WPI_VictorSPX intake = new WPI_VictorSPX(Ports.kIntakeCANID);

  private final DigitalInput ballSensor = new DigitalInput(Ports.kBallSensorPort);
  
  /** Creates a new ScoopyScoop. */
  public ScoopyScoop() {

  }
  public void intakeIntake(){intake.set(MotorControllerValues.kIntakeValue);}
  public void intakeReverse(){intake.set(-MotorControllerValues.kIntakeValue);}
  public void intakeSTOP(){intake.stopMotor();}
  
  public void rollerIntake(){belt.set(MotorControllerValues.kRollerValues);}
  public void rollerShoot(){belt.set(-MotorControllerValues.kRollerValues);}
  public void rollerSTOP(){belt.stopMotor();}

  public void autoFeedShooter(BooleanSupplier ready){
    if(ready.getAsBoolean()){rollerShoot();}else{rollerSTOP();}
  }

  public boolean getBallSensor(){return ballSensor.get();}
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
