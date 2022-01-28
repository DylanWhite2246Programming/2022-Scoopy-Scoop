// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.MotorControllerValues;
import frc.robot.Constants.Ports;
import frc.robot.commands.FacePose;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.PowerAndPneumatics;
import frc.robot.subsystems.ScoopyScoop;
import frc.robot.subsystems.Vision;
import frc.robot.team2246.Drivestation;
import frc.robot.team2246.NetworktableHandeler;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private final ScoopyScoop scoop = new ScoopyScoop();
  private final Vision vision = new Vision();
  
  private final Drivestation controller = new Drivestation(Ports.kUSBPorts);
  private final NetworktableHandeler tableButtons = new NetworktableHandeler();

  private final InstantCommand setAutoModeOn = new InstantCommand(()->power.setAutoMode(true), power);
  private final InstantCommand setAutoModeOff = new InstantCommand(()->power.setAutoMode(false), power);

  private final InstantCommand turnOnVision = new InstantCommand(()->vision.setDriverMode(false), vision);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    drivetrain.setDefaultCommand(
      new RunCommand(()->drivetrain.drive(controller.getLeftY(),controller.getRightX()), drivetrain)
    );
    scoop.setDefaultCommand(
      new ParallelCommandGroup(
        new RunCommand(()->scoop.shooterSTOP(), scoop),
        new RunCommand(()->scoop.rollerSTOP(), scoop)
      )
    );
    new RunCommand(()->vision.setOveride(tableButtons.getOverideAutoPipe(), tableButtons.getManualPipe()), vision);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Network buttons
    //left stick
    controller.ls0
    .whileHeld(new RunCommand(()->drivetrain.setMaxOutput(1), drivetrain), true)
    .whenPressed(new RunCommand(()->drivetrain.setMaxOutput(.75), drivetrain), true);
    controller.ls1.whileHeld(
      new ParallelCommandGroup(
        new ConditionalCommand(
          new ConditionalCommand( //run when first sensor is true
          new RunCommand(()->scoop.rollerIntake(), scoop) //run when a ball comes in
          .withInterrupt(scoop::getSecondSensor),
          new RunCommand(()->scoop.rollerSTOP(), scoop), //run while there are no balls coming in
          scoop::getEntrySensor
          ), 
          new RunCommand(()->scoop.rollerIntake(), scoop), //run when first sensor is false
          scoop::getFirstSensor
          ),
          //run the hole time
          new RunCommand(()->scoop.intake(), scoop),
          setAutoModeOn
          ), 
          false
          );
    //POV
    new Button(()->controller.leftPovEquals(0))
      .whenPressed(new InstantCommand(()->climber.extendBackSolenoid()));
    new Button(()->controller.leftPovEquals(180))
      .whenPressed(new InstantCommand(()->climber.retrackBackSolenoid()));
    //rightstick
    controller.rs0
      .whileHeld(new RunCommand(()->drivetrain.setMaxOutput(.3), drivetrain), true)
      .whenReleased(new RunCommand(()->drivetrain.setMaxOutput(.75), drivetrain), true);
    controller.rs2
      .whileHeld(new FacePose(new Translation2d(0, 0), controller::getLeftY, drivetrain)
        .alongWith(setAutoModeOn), 
      false);
    controller.rs3
      .whileHeld(new PIDCommand(
        drivetrain.getTurnController(), 
        vision.getResults().getBestTarget()::getYaw, 
        0.0, 
        output->{drivetrain.computerDrive(controller.getLeftY(),output);}, 
        drivetrain)
          .alongWith(turnOnVision,setAutoModeOn), 
        false
      ).whenReleased(
        new InstantCommand(()->vision.setDriverMode(true), vision)
          .alongWith(setAutoModeOff)
      );

    //switch row 0
    controller.s00.whileHeld(new InstantCommand(()->scoop.shoot(MotorControllerValues.kShooterVelocity/3), scoop));//prime shooter
    controller.s01.whileHeld(
      new InstantCommand(
        ()->lift.aim(
          drivetrain.getDistanceTo(
            new Translation2d(0, 0))), 
        lift
      )
    );
    //switch row 1
    //button row 0
    controller.b00.whileHeld(new InstantCommand(()->scoop.rollerIntake(), scoop));
    controller.b01.whileHeld(new InstantCommand(()->scoop.rollerShoot(), scoop));
    controller.b02.whileHeld(new InstantCommand(()->climber.extendLifterSolenoid(), climber));
    //button row 1
    controller.b10.whileHeld(new InstantCommand(()->scoop.intake(), scoop));
    controller.b11.whileHeld(new InstantCommand(()->scoop.shoot(MotorControllerValues.kShooterVelocity), scoop));
    controller.b12.whileHeld(new InstantCommand(()->climber.retrackLifterSolenoid(), climber));
    //button row 2
    controller.b20.whileHeld(
      new ParallelCommandGroup(
        new ConditionalCommand(
          new ConditionalCommand( //run when first sensor is true
            new RunCommand(()->scoop.rollerIntake(), scoop) //run when a ball comes in
              .withInterrupt(scoop::getSecondSensor),
            new RunCommand(()->scoop.rollerSTOP(), scoop), //run while there are no balls coming in
            scoop::getEntrySensor
          ), 
          new RunCommand(()->scoop.rollerIntake(), scoop), //run when first sensor is false
          scoop::getFirstSensor
        ),
        //run the hole time
        new RunCommand(()->scoop.intake(), scoop),
        setAutoModeOn
      ), 
      false
    );
    controller.b21.whileHeld(
      new ConditionalCommand(
        new SequentialCommandGroup(
          new RunCommand(()->scoop.shoot(MotorControllerValues.kShooterVelocity), scoop),
          new RunCommand(()->scoop.rollerShoot(), scoop)), 
        new SequentialCommandGroup(
          new RunCommand(()->scoop.shoot(MotorControllerValues.kShooterVelocity), scoop),
          new RunCommand(()->scoop.rollerSTOP(), scoop)), 
        scoop::shooterAtSetpoint)
          .alongWith(setAutoModeOn), 
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
