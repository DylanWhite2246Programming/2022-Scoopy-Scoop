// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FacePose extends PIDCommand {
  /** Creates a new RotateToAngle. */
  public FacePose(Translation2d pose, DoubleSupplier x, Drivetrain drivetrain)  {
    super(
      // The controller that the command will use
      drivetrain.getTurnController(),
      // This should return the measurement
      drivetrain::getAbsoluteHeading,
      // This should return the setpoint (can also be a constant)
      ()->Math.atan2(
        drivetrain.getPose().getTranslation().minus(pose).getX(),
        drivetrain.getPose().getTranslation().minus(pose).getY()
      ),
      // This uses the output
      output -> {
        drivetrain.computerDrive(x.getAsDouble(), output);
      }
    );
    addRequirements(drivetrain);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
