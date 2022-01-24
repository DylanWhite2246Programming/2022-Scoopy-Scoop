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
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.LifterConstants;
import frc.robot.Constants.Ports;

public class Lifter extends ProfiledPIDSubsystem {
  private final CANSparkMax motor = new CANSparkMax(Ports.kLifterCANID, MotorType.kBrushless);
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
    motor.getEncoder().setPosition(-LifterConstants.kOffSet);
    motor.getEncoder().setPositionConversionFactor(LifterConstants.kConversionFactor);
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
    var outputVolts = a+output;
    if(toplimit.get()&&outputVolts<=0){
      motor.setVoltage(outputVolts);
    }else if(bottomlimit.get()&&outputVolts>=0){
      motor.setVoltage(outputVolts);
    }else{
      if(
        getMeasurement()>-LifterConstants.kOffSet+.1
        && getMeasurement()<1.5708-LifterConstants.kOffSet
      ){
        motor.setVoltage(outputVolts);
      }
    }
  }
  @Override
  public double getMeasurement() {
    return motor.getEncoder().getPosition();
  }
}
