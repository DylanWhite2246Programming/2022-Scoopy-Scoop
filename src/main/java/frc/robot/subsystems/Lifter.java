// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.LifterConstants;
import frc.robot.Constants.Ports;

public class Lifter extends ProfiledPIDSubsystem {
  private final WPI_VictorSPX motor = new WPI_VictorSPX(Ports.kLifterCANID);
  private final DutyCycleEncoder encoder = new DutyCycleEncoder(Ports.kLifterEncoderPort);
  private final DigitalInput toplimit = new DigitalInput(Ports.kTopLimitPort);
  private final DigitalInput bottomlimit = new DigitalInput(Ports.kBottomLimitPort);
  //TODO find these values
  private final ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0, 0);
  /** Creates a new Lifter. */
  public Lifter() {
    super(
      // The ProfiledPIDController used by the subsystem
      //TODO change values
      new ProfiledPIDController(
        LifterConstants.kP,
        LifterConstants.kI,
        LifterConstants.kD,
        // The motion profile constraints
        new TrapezoidProfile.Constraints(
          LifterConstants.kMaxVelocity, LifterConstants.kMaxAcceleration
        )
      )
    );
    encoder.setDistancePerRotation(2*Math.PI);
    encoder.reset();
    motor.setSafetyEnabled(false);
  }
  //TODO change values
  private double calculateLaunchAngle(double distance){
    //return Math.asin((distance*Constants.kG)/Math.pow(ScoopConstants.velocity,2));
    return 0;
  }
  public void aim(double distance){
    getController().setPID(
      LifterConstants.kP, 
      LifterConstants.kI, 
      LifterConstants.kD
    );
    if(isEnabled()==false){enable();}
    if(
      distance>LifterConstants.kMaxDistance
      ||distance<LifterConstants.kClosestDistance
    ){
      setGoal(calculateLaunchAngle(distance)); 
    }
  }
  /**
   * @param angle angle value in radians
   */
  public void setAngle(double angle, boolean climbing){
    //TODO CHANGE
    if(climbing){
      getController().setPID(0, 0, 0);
    }else{
      getController().setPID(
        LifterConstants.kP, 
        LifterConstants.kI, 
        LifterConstants.kD
      );
    }
    if(isEnabled()==false){enable();}
    setGoal(angle);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    var a = feedforward.calculate(setpoint.position, setpoint.velocity);
    if(toplimit.get()&&(a+output)<=0){
      motor.setVoltage(a+output);
    }else if(bottomlimit.get()&&(a+output>=0)){
      motor.setVoltage(a+output);
    }else{
      motor.setVoltage(a+output);
    }

    if(
      getMeasurement()>-LifterConstants.kOffSet+.1
      && getMeasurement()<1.5708-LifterConstants.kOffSet
    ){
      motor.setVoltage(a+output);
    }
  }
  @Override
  public double getMeasurement() {
    return encoder.getDistance()-LifterConstants.kOffSet;
  }
}
