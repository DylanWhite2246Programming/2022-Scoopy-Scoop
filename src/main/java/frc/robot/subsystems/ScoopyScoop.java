// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorControllerValues;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ScoopConstants;

public class ScoopyScoop extends SubsystemBase {

  private final WPI_VictorSPX rollers = new WPI_VictorSPX(Ports.kRollerCANID);
  private final WPI_VictorSPX leftShooter = new WPI_VictorSPX(Ports.kLeftShooterCANID);
  private final WPI_VictorSPX rightShooter = new WPI_VictorSPX(Ports.kRightShooterCANID);
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
    leftShooter.setInverted(ScoopConstants.kLeftMotorInverted);
    rightShooter.setInverted(ScoopConstants.kRightMotorInverted);
  }

  public void Shoot(){
    leftShooter.set(-MotorControllerValues.kIntakeValue);
    rightShooter.set(-MotorControllerValues.kIntakeValue);
  }
  /**
   * @return left shooter speed in radians per second
   */
  public double getLeftVelocity(){
    return leftEncoder.getRate();
  }
  /**
   * @return right shooter speed in radians per second
   */
  public double getRightVelocity(){
    return rightEncoder.getRate();
  }
  /**
   * @param setpoint desired velocity in radians per second
   */
  public void shoot(double setpoint){
    leftPid.setSetpoint(setpoint);
    rightPid.setSetpoint(setpoint);
    leftShooter.setVoltage(
      leftFeedForward.calculate(setpoint)
        +leftPid.calculate(getLeftVelocity())
    );
    rightShooter.setVoltage(
      rightFeedforward.calculate(setpoint)
        +rightPid.calculate(getRightVelocity())
    );
  }
  public void rollerIntake(){
    rollers.set(MotorControllerValues.kRollerValues);
  }
  public void rollerShoot(){
    rollers.set(-MotorControllerValues.kRollerValues);
  }
  public void rollerSTOP(){
    rollers.stopMotor();
  }
  public void shooterStop(){
    leftShooter.stopMotor();
    rightShooter.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
