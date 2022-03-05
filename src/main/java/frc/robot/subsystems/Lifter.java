// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.LifterConstants;
import frc.robot.Constants.Ports;

public class Lifter extends ProfiledPIDSubsystem {
  private RelativeEncoder encoder;
  private final CANSparkMax motor = new CANSparkMax(Ports.kLifterCANID, MotorType.kBrushless);
  private final DigitalInput toplimit = new DigitalInput(Ports.kTopLimitPort);
  private final DigitalInput bottomlimit = new DigitalInput(Ports.kBottomLimitPort);
  //TODO find these values
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
      ),0
    );
    encoder=motor.getEncoder();
    encoder.setPositionConversionFactor(LifterConstants.kConversionFactor);
    encoder.setVelocityConversionFactor(LifterConstants.kVelConversionFactor);
    encoder.setPosition(LifterConstants.kOffSet);
    getController().setTolerance(LifterConstants.kTolerence);
    motor.setInverted(LifterConstants.kInverted);
  }
  //TODO change values
  //private double calculateLaunchAngle(double distance){
  //  //return Math.asin((distance*Constants.kG)/Math.pow(ScoopConstants.velocity,2));
  //  return 0;
  //}
  //public void aim(double distance){
  //  getController().setPID(
  //    LifterConstants.kP, 
  //    LifterConstants.kI, 
  //    LifterConstants.kD
  //  );
  //  if(isEnabled()==false){enable();}
  //  if(calculateLaunchAngle(distance)>LifterConstants.kIntakeClerence){
  //    setGoal(calculateLaunchAngle(distance));
  //  }
  //}
  /**
   * @param angle angle value in radians
   */
  public void setAngle(double angle, boolean climbing){
    if(climbing){
      //TODO CHANGE
      getController().setPID(0, 0, 0);
    }else{
      getController().setPID(
        LifterConstants.kP, 
        LifterConstants.kI, 
        LifterConstants.kD
      );
    }
    if(isEnabled()==false){enable();}
    setGoal(new TrapezoidProfile.State(angle, 0));
    //setGoal(angle);
  }

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
  }
  @Override
  public double getMeasurement() {
    return encoder.getPosition();
  }
  @Override
  public void periodic(){
    //System.out.println(getMeasurement());
    SmartDashboard.putNumber("Lift Position", getMeasurement());
    SmartDashboard.putNumber("Output Voltage", motor.getAppliedOutput());
    SmartDashboard.putBoolean("Top Limit", toplimit.get());
    SmartDashboard.putBoolean("Bottom Limit", bottomlimit.get());
    SmartDashboard.putBoolean("At Goal", getController().atGoal());
  }
}
