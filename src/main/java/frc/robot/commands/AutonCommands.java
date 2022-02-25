// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutonCommandConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;
/** Add your docs here. */
public class AutonCommands {
    public static RamseteCommand ramseteCommandGenerator(Drivetrain drivetrain, Trajectory trajectory){
        return new RamseteCommand(
            trajectory, 
            drivetrain::getPose, 
            AutonCommandConstants.controller, 
            AutonCommandConstants.feedForward, 
            DrivetrainConstants.KINEMATICS, 
            drivetrain::getWheelSpeeds, 
            AutonCommandConstants.leftController, 
            AutonCommandConstants.rightController, 
            drivetrain::driveVolts, 
            drivetrain
        );
    }
}