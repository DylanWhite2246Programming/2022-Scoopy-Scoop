// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.Ports;
import frc.robot.commands.RotateToGoal;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ScoopyScoop;
import frc.robot.team2246.Drivestation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private final Climber climber = new Climber();
  private final ScoopyScoop scoop = new ScoopyScoop();

  private final Drivestation controller = new Drivestation(Ports.kUSBPorts);
  //private final Joystick controller = new Joystick(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    drivetrain.setDefaultCommand(
      new RunCommand(()->drivetrain.drive(controller.getLeftY(),controller.getRightX()), drivetrain)
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new Button(()->controller.rightPovEquals(0))
      .whenPressed(new InstantCommand(()->climber.extendBackSolenoid(), climber));
    new Button(()->controller.rightPovEquals(180))
      .whenPressed(new InstantCommand(()->climber.retrackBackSolenoid(), climber));
    controller.ls0
      .whileHeld(new RunCommand(()->drivetrain.setMaxOutput(1), drivetrain), true)
      .whenPressed(new RunCommand(()->drivetrain.setMaxOutput(.75), drivetrain), true);
    controller.rs0
      .whileHeld(new RunCommand(()->drivetrain.setMaxOutput(.3), drivetrain), true)
      .whenReleased(new RunCommand(()->drivetrain.setMaxOutput(.75), drivetrain), true);
    controller.rs1
      .whileHeld(new RotateToGoal(drivetrain, controller::getLeftY), false);
    
    controller.b20
      .whileHeld(
        new ParallelCommandGroup(
          new ConditionalCommand(
            new ConditionalCommand( //run when first sensor is true
              new RunCommand(()->scoop.rollerIntake(), scoop) //run when a ball comes in
                .withInterrupt(scoop::getSecondSensor)
                .withTimeout(3), 
              new RunCommand(()->scoop.rollerSTOP(), scoop), //run while there are no balls coming in
              scoop::getEntrySensor
            ), 
            //run when first sensor is false
            new RunCommand(()->scoop.rollerIntake(), scoop), 
            scoop::getFirstSensor
          ),
          new RunCommand(()->scoop.intakeShooter(), scoop) //run the hole time
        ), 
        false
      );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
