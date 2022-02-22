// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.LifterConstants;
import frc.robot.Constants.Ports;

public class LifterTrapezoidal extends TrapezoidProfileSubsystem {
  private final CANSparkMax motor = new CANSparkMax(Ports.kLifterCANID, MotorType.kBrushless);
  private SparkMaxPIDController pid;
  private ArmFeedforward feedForward; //= new ArmFeedforward(ks, kcos, kv, ka);
  private DigitalInput topLimit, bottomLimit;
  /** Creates a new LifterTrapezoidal. */
  public LifterTrapezoidal() {
    super(
      // The constraints for the generated profiles
      new TrapezoidProfile.Constraints(0, 0),
      // The initial position of the mechanism
      LifterConstants.kOffSet
    );
    pid = motor.getPIDController();
    motor.getPIDController().setP(LifterConstants.kP);
    topLimit = new DigitalInput(Ports.kTopLimitPort);
    bottomLimit = new DigitalInput(Ports.kBottomLimitPort);
  }

  @Override
  protected void useState(TrapezoidProfile.State state) {
    // Use the computed profile state here.
  }
}
