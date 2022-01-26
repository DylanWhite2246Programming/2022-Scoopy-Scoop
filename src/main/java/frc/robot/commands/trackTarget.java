// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class trackTarget extends CommandBase {
  Drivetrain drivetrain;
  Vision vision;
  Double magnitude;
  /** Creates a new trackTarget. */
  public trackTarget(Vision cam, Drivetrain drive, DoubleSupplier x) {
    addRequirements(drive);
    withTimeout(5);
    drivetrain=drive;
    vision=cam;
    magnitude=x.getAsDouble();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(DriverStation.getAlliance()==Alliance.Blue){vision.setPipe(1);}
    else if(DriverStation.getAlliance()==Alliance.Red){vision.setPipe(2);}
    else{vision.setPipe(0);}
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(
      magnitude, 
      -drivetrain.getTurnController().calculate(
        vision.getResults().getBestTarget().getYaw(), 
        0
      )
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
