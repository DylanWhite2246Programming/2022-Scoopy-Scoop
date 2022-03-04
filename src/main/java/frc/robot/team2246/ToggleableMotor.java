// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.team2246;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ToggleableMotor extends SubsystemBase {
  private WPI_VictorSPX motor;
  private double speed;
  /**
   * creates a victor spx motor controller with the given parameters assumed to not be reversed
   * @param canid can id to construct the victor spx
   * @param speed the speed to be used both forward and back
   * @param inverted wether or not the motor is inverted
   */
  public ToggleableMotor(int canid, double speed, boolean inverted) {
    motor = new WPI_VictorSPX(canid);
    this.speed=speed;
    motor.setInverted(inverted);
  }
  /**
   * creates a victor spx motor controller with the given parameters assumed to not be reversed
   * @param canid can id to construct the victor spx
   * @param speed the speed to be used both forward and back
   */
  public ToggleableMotor(int canid, double speed){
    motor = new WPI_VictorSPX(canid);
    this.speed=speed;
  }

  public RunCommand forward = new RunCommand(()->{motor.set(speed);}, this);
  public RunCommand reverse = new RunCommand(()->{motor.set(-speed);}, this);
  public RunCommand stop = new RunCommand(()->{motor.stopMotor();}, this);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}