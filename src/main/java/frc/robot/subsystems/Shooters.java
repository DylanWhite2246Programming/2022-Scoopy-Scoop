// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ScoopConstants;

public class Shooters extends SubsystemBase {
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
    = new SimpleMotorFeedforward(-.59328, .98916, 3.6831);
  
  private final SimpleMotorFeedforward rightFeedforward
    = new SimpleMotorFeedforward(-.63413, 1.0584, .67737);

  private final PIDController leftPid
    = new PIDController(.018289, 0, 0);
  
  private final PIDController rightPid
    = new PIDController(.018289, 0, 0);

  /** Creates a new Shooters. */
  public Shooters() {
    leftEncoder.setDistancePerPulse(ScoopConstants.kDistancePerPulse);
    rightEncoder.setDistancePerPulse(ScoopConstants.kDistancePerPulse);
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
  public void setVolts(double volts){
    leftShooter.setVoltage(volts);
    rightShooter.setVoltage(volts);
  }
  public boolean atSetpoint(){
    return leftPid.atSetpoint()&&rightPid.atSetpoint();
  }
  public void STOP(){
    leftShooter.stopMotor();
    rightShooter.stopMotor();
    shoot(0);
  }
  public void intake(){
    setVolts(6);
    //leftShooter.set(MotorControllerValues.kShooterIntakeValue);
    //rightShooter.set(MotorControllerValues.kShooterIntakeValue);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Shooter Vel", getLeftVelocity());
    SmartDashboard.putNumber("Left Volts", leftShooter.getMotorOutputVoltage());
    SmartDashboard.putNumber("Right Shooter Vel", getRightVelocity());
    SmartDashboard.putNumber("Right Volts", rightShooter.getMotorOutputVoltage());
    // This method will be called once per scheduler run
  }
}
