// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.LifterConstants;
import frc.robot.Constants.Ports;

public class Lifter extends ProfiledPIDSubsystem {
  private final DutyCycleEncoder encoder = new DutyCycleEncoder(Ports.kLifterEncoderPort);
  private final CANSparkMax motor = new CANSparkMax(Ports.kLifterCANID, MotorType.kBrushless);

  private final DigitalInput toplimit = new DigitalInput(Ports.kTopLimitPort);
  private final DigitalInput bottomlimit = new DigitalInput(Ports.kBottomLimitPort);

  private final ArmFeedforward feedforward = new ArmFeedforward(
    LifterConstants.kS, 
    LifterConstants.kG, 
    LifterConstants.kV, 
    LifterConstants.kA
  );
  /** Creates a new Lifter. */
  public Lifter() {
    super(
      // The ProfiledPIDController used by the subsystem
      new ProfiledPIDController(
        LifterConstants.kP,
        LifterConstants.kI,
        LifterConstants.kD,
        // The motion profile constraints
        new TrapezoidProfile.Constraints(
          LifterConstants.kMaxVelocity, LifterConstants.kMaxAcceleration
        )
      ),LifterConstants.kOffSet
    );
    encoder.setDistancePerRotation(2*Math.PI);
    encoder.reset();
    getController().setTolerance(LifterConstants.kTolerence);
    motor.setInverted(LifterConstants.kInverted);
  }
  //TODO change values

  public void setMotorVoltage(double x){
    //if(toplimit.get()&&x<=0){
    //  motor.setVoltage(x);
    //}else if(bottomlimit.get()&&x>=0){
    //  motor.setVoltage(x);
    //}else{
    //  motor.stopMotor();
    //}
    motor.setVoltage(x);
  }
  public void STOP(){
    disable();
    motor.stopMotor();
  }
  public boolean atSetpoint(){return getController().atSetpoint();}
  
  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double feedForward = feedforward.calculate(setpoint.position, setpoint.velocity);
    motor.setVoltage(feedForward+output);
    //System.out.println("Test");
    //System.out.println(feedforward);
  }
  @Override
  public double getMeasurement() {
    return encoder.getDistance()+LifterConstants.kOffSet;
  }
  @Override
  public void periodic(){
    super.periodic();
    //System.out.println(getMeasurement());
    SmartDashboard.putNumber("Lift Position", getMeasurement());
    SmartDashboard.putNumber("Output Voltage", motor.getAppliedOutput());
    SmartDashboard.putNumber("Setpoint", getController().getSetpoint().position);
    SmartDashboard.putBoolean("Top Limit", toplimit.get());
    SmartDashboard.putBoolean("Bottom Limit", bottomlimit.get());
    SmartDashboard.putBoolean("At Goal", getController().atGoal());
  }
}
