// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutonTrajectorys;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.ScoopyScoop;

public class Auton1 extends CommandBase {
  private Drivetrain drivetrain;
  private ScoopyScoop scoop;
  private Lifter lifter;
  /** Creates a new Auton1. */
  public Auton1(Drivetrain drivetrain, ScoopyScoop scoop, Lifter lifter) {
    this.drivetrain=drivetrain;
    this.scoop=scoop;
    this.lifter=lifter;
    addRequirements(drivetrain,scoop,lifter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    AutonCommands.ramseteCommandGenerator(drivetrain, AutonTrajectorys.kAuton1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while(drivetrain.getDistanceTo(AutonTrajectorys.kAuton1.get)){

    }
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
