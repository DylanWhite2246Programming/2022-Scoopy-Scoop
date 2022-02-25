// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorControllerValues;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ScoopConstants;

public class ScoopyScoop extends SubsystemBase {

  private final WPI_VictorSPX belt = new WPI_VictorSPX(Ports.kBeltCANID);
  private final WPI_VictorSPX leftShooter = new WPI_VictorSPX(Ports.kLeftShooterCANID);
  private final WPI_VictorSPX rightShooter = new WPI_VictorSPX(Ports.kRightShooterCANID);
  private final WPI_VictorSPX intake = new WPI_VictorSPX(Ports.kIntakeCANID);
  private final Encoder leftEncoder = new Encoder(
    Ports.kLeftShooterEncoderPorts[0], 
    Ports.kLeftShooterEncoderPorts[1], 
    ScoopConstants.kLeftEncoderReversed
  );
  private final Encoder rightEncoder = new Encoder(
    Ports.kRightShooterEncoderPorts[0], 
    Ports.kRightShooterEncoderPorts[1], 
    ScoopConstants.kRightEncoderReversed
  );
  private final DigitalInput ballSensor = new DigitalInput(Ports.kBallSensorPort);

  //TODO change
  private final SimpleMotorFeedforward leftFeedForward 
    = new SimpleMotorFeedforward(0, 0, 0);
  
  private final SimpleMotorFeedforward rightFeedforward
    = new SimpleMotorFeedforward(0, 0, 0);

  private final PIDController leftPid
    = new PIDController(0, 0, 0);
  
  private final PIDController rightPid
    = new PIDController(0, 0, 0);
  
  /** Creates a new ScoopyScoop. */
  public ScoopyScoop() {
    leftEncoder.setDistancePerPulse(ScoopConstants.kDistancePerPulse);
    rightEncoder.setDistancePerPulse(ScoopConstants.kDistancePerPulse);
    //TODO create constants
    leftEncoder.setReverseDirection(ScoopConstants.kLeftEncoderReversed);
    rightEncoder.setReverseDirection(ScoopConstants.kRightEncoderReversed);
    leftShooter.setInverted(ScoopConstants.kLeftMotorInverted);
    rightShooter.setInverted(ScoopConstants.kRightMotorInverted);
  }

  /**
   * @return left shooter speed in meters per second
   */
  public double getLeftVelocity(){
    return leftEncoder.getRate();
  }
  /**
   * @return right shooter speed in meters per second
   */
  public double getRightVelocity(){
    return rightEncoder.getRate();
  }
  /**
   * @param setpoint desired velocity in meters per second
   */
  public void shoot(double setpoint){
    leftShooter.setVoltage(
      leftFeedForward.calculate(setpoint)
        +leftPid.calculate(getLeftVelocity(), setpoint)
    );
    rightShooter.setVoltage(
      rightFeedforward.calculate(setpoint)
        +rightPid.calculate(getRightVelocity(), setpoint)
    );
  }
  public boolean shooterAtSetpoint(){
    return leftPid.atSetpoint()&&
    rightPid.atSetpoint();
  }
  public void shooterSTOP(){
    leftShooter.stopMotor();
    rightShooter.stopMotor();
    shoot(0);
  }
  public void shooterIntake(){
    leftShooter.set(-MotorControllerValues.kIntakeValue);
    rightShooter.set(-MotorControllerValues.kIntakeValue);
  }
  public void intakeIntake(){intake.set(MotorControllerValues.kIntakeValue);}
  public void intakeReverse(){intake.set(-MotorControllerValues.kIntakeValue);}
  public void intakeSTOP(){intake.stopMotor();}
  public void rollerIntake(){
    belt.set(MotorControllerValues.kRollerValues);
  }
  public void rollerShoot(){
    belt.set(-MotorControllerValues.kRollerValues);
  }
  public void rollerSTOP(){
    belt.stopMotor();
  }

  public void autoIntake(){
    shooterIntake();//intake shooter the hole time
    intakeIntake();//run intake the hole time
    if(getBallSensor()){
      rollerSTOP();
    }else{
      rollerIntake();//intake rollers when all sensors are false
    }
  }
  public void autoFeedShooter(){
    if(shooterAtSetpoint()){
      rollerShoot();
    }else{
      rollerSTOP();
    }
  }

  public boolean getBallSensor(){return ballSensor.get();}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
