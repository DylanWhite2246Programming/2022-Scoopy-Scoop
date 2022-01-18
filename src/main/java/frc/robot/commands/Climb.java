// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.LifterConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Lifter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Climb extends SequentialCommandGroup {
  /** Creates a new Climb. */
  public Climb(Climber climber, Lifter lifter, Drivetrain drivetrain) {
    drivetrain.stop();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new InstantCommand(()->climber.extendBackSolenoid(), climber),
      //new WaitCommand(.75),
      //AutonCommands.moveToClimb(drivetrain),
      new InstantCommand(()->climber.retrackBackSolenoid(), climber),
      new WaitCommand(1.8),
      new InstantCommand(()->lifter.setAngle(LifterConstants.kClimbPosition, true), lifter),
      new WaitUntilCommand(lifter.getController()::atGoal),
      new InstantCommand(()->climber.extendLifterSolenoid(), climber),
      new WaitCommand(.5),
      new InstantCommand(()->lifter.disable(), lifter),
      new WaitCommand(1),
      new InstantCommand(()->climber.retrackLifterSolenoid(), climber),
      new WaitCommand(.75),
      new InstantCommand(()->climber.extendBackSolenoid(), climber),
      //new WaitCommand(.75),
      //new InstantCommand(()->lifter.setAngle(LifterConstants.kSecondClimbingPosition, true), lifter),
      //new WaitUntilCommand(lifter.getController()::atGoal),lk
      //new InstantCommand(()->climber.retrackBackSolenoid(), climber),
      new InstantCommand(()->lifter.disable(), lifter)
    );
  }
  
}
