// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.MotorControllerValues;
import frc.robot.subsystems.Shooters;

public class ShooterStart extends CommandBase {
  Shooters shooters;
  /** Creates a new ShooterStart. */
  public ShooterStart(Shooters shooters) {
    this.shooters=shooters;
    addRequirements(this.shooters);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooters.shoot(MotorControllerValues.kShooterVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooters.shoot(0);
    shooters.STOP();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}