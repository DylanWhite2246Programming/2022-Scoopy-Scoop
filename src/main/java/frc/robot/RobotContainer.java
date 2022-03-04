// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import frc.robot.Constants.LifterConstants;
import frc.robot.Constants.MotorControllerValues;
import frc.robot.Constants.Ports;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.PowerAndPneumatics;
import frc.robot.subsystems.Shooters;
import frc.robot.subsystems.Vision;
import frc.robot.team2246.Drivestation;
import frc.robot.team2246.NetworktableHandeler;
import frc.robot.team2246.ToggleableMotor;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final Climber climber = new Climber();
  private final Drivetrain drivetrain = new Drivetrain();
  private final Lifter lift = new Lifter();
  private final PowerAndPneumatics power = new PowerAndPneumatics();
  private final Shooters shooters = new Shooters();
  private final Vision vision = new Vision();

  private final ToggleableMotor intake = new ToggleableMotor(Ports.kIntakeCANID, MotorControllerValues.kIntakeValue);
  private final ToggleableMotor belt = new ToggleableMotor(Ports.kBeltCANID, 1);
  
  //private final Drivestation controller = new Drivestation(Ports.kUSBPorts);
  private final NetworktableHandeler tableButtons = new NetworktableHandeler();
  private final Joystick stick = new Joystick(0);

  private final BooleanSupplier shooterReady = ()->{return shooters.atSetpoint()&&lift.getController().atSetpoint();};
  private final ParallelCommandGroup intakeBoth = new ParallelCommandGroup(intake.forward,belt.forward);

  private final InstantCommand startShooter = new InstantCommand(()->{shooters.shoot(MotorControllerValues.kShooterVelocity);}, shooters);
  
  private final InstantCommand extendArm = new InstantCommand(()->{climber.extendBackSolenoid();}, climber);
  private final InstantCommand retrackArm = new InstantCommand(()->{climber.retrackBackSolenoid();}, climber);

  //TODO change
  private final InstantCommand startLifter = new InstantCommand(()->{lift.aim(0);}, lift);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    //drivetrain.setDefaultCommand(
    //  new RunCommand(()->drivetrain.drive(controller.getLeftY(),controller.getRightX()), drivetrain)
    //);
    //lift.setDefaultCommand(new InstantCommand(()->{lift.setAngle(LifterConstants.kOffSet, false);}, lift));
    //scoop.setDefaultCommand(
    //  new InstantCommand(()->{scoop.intakeSTOP();scoop.rollerSTOP();}, scoop)
    //);
    //shooters.setDefaultCommand(new InstantCommand(()->{shooters.STOP();}, shooters));
    vision.setDefaultCommand(
      new RunCommand(()->{vision.setOveride(tableButtons.getOverideAutoPipe(), tableButtons.getManualPipe());}, vision)
    );
    lift.setDefaultCommand(
      new RunCommand(()->{lift.setMotorVoltage(12*stick.getRawAxis(2));}, lift)
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //new Button(RobotController::getUserButton)
    //  .whenPressed(new InstantCommand(()->{climber.toggleBackSolenoid();}, climber));
    //Network
    //Left Stick
    //controller.ls10.whenPressed(extendArm);
    //controller.ls9.whenPressed(retrackArm);
    new Button(()->{return stick.getRawButton(3);}).whenPressed(extendArm);
    new Button(()->{return stick.getRawButton(2);}).whenPressed(retrackArm);
    //Right Stick
    //Switches
    //Buttons
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
