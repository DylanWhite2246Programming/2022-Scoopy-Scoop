// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.MotorControllerValues;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.PowerAndPneumatics;
import frc.robot.subsystems.Shooters;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final SlewRateLimiter leftlimiter = new SlewRateLimiter(.8);
  private final SlewRateLimiter rightlimiter = new SlewRateLimiter(1.2);
  
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private final Indexer indexer = new Indexer();
  private final PowerAndPneumatics power = new PowerAndPneumatics();
  private final Shooters shooters = new Shooters();

  //private final NetworktableHandeler tableButtons = new NetworktableHandeler();

  private final Joystick left = new Joystick(0);
  private final Joystick right = new Joystick(1);
  private final Joystick bb = new Joystick(2);

  private final InstantCommand intakeShooter = new InstantCommand(()->shooters.setVolts(MotorControllerValues.kShooterIntakeValue), shooters);
  private final InstantCommand startShooter = new InstantCommand(()->{shooters.shoot(MotorControllerValues.kShooterVelocity);}, shooters);
  private final InstantCommand stopShooter = new InstantCommand(()->{shooters.STOP();}, shooters);

  private final InstantCommand intake = new InstantCommand(()->indexer.intakeForward(), indexer);
  private final InstantCommand reverseIntake = new InstantCommand(()->indexer.beltReverse(), indexer);
  private final InstantCommand stopIntake = new InstantCommand(()->indexer.intakeSTOP(), indexer);

  private final InstantCommand belt = new InstantCommand(()->indexer.beltForward(), indexer);
  private final InstantCommand reverseBelt = new InstantCommand(()->indexer.beltReverse(), indexer);
  private final InstantCommand stopBelt = new InstantCommand(()->indexer.beltSTOP(), indexer);

  private final InstantCommand scoopUp = new InstantCommand(()->power.scoopUp(), power);
  private final InstantCommand scoopDown = new InstantCommand(()->power.scoopDown(), power);

  private final InstantCommand climbUp = new InstantCommand(()->power.climberUp(), power);
  private final InstantCommand climbDown = new InstantCommand(()->power.climbDown(), power);

  SequentialCommandGroup taxiBack(double time, double speed){
    return new SequentialCommandGroup(
      new RunCommand(()->drivetrain.computerDrive(speed, 0), drivetrain).withTimeout(time),
      new InstantCommand(()->{drivetrain.STOP();drivetrain.brake();}, drivetrain),
      new WaitCommand(1),
      new InstantCommand(()->drivetrain.idle(), drivetrain)
    );
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    drivetrain.setDefaultCommand(
      //new RunCommand(()->drivetrain.drive(controller.getLeftY(),controller.getRightX()), drivetrain)
      
      new RunCommand(()->{drivetrain.drive(
        -leftlimiter.calculate(Constants.joystickTune(left.getY())), 
        rightlimiter.calculate(Constants.joystickTune(.6*right.getX()))
      );}, drivetrain)
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //B00
    new Button(()->bb.getRawButton(1)).whenPressed(intake).whenReleased(stopIntake);
    //B01
    new Button(()->bb.getRawButton(2)).whenPressed(belt).whenReleased(stopBelt);
    //B02
    new Button(()->bb.getRawButton(3)).whenPressed(intakeShooter).whenReleased(stopShooter);
    //B03
    new Button(()->bb.getRawButton(4)).whenPressed(climbUp);
    //B10
    new Button(()->bb.getRawButton(5)).whenPressed(reverseIntake).whenReleased(stopIntake);
    //B11
    new Button(()->bb.getRawButton(6)).whenPressed(reverseBelt).whenReleased(stopBelt);
    //B12
    new Button(()->bb.getRawButton(7)).whenPressed(startShooter).whenReleased(stopShooter);
    //B13
    new Button(()->bb.getRawButton(8)).whenPressed(climbDown);
    //B20
    new Button(()->bb.getRawButton(9)).whenPressed(scoopUp);
    //B21
    new Button(()->bb.getRawButton(10)).whenPressed(scoopDown);
    //S23
    //S24
    new Button(()->bb.getRawButton(12)).whileHeld(
      new InstantCommand(()->power.compressorOn(), power)
    ).whenReleased(
      new InstantCommand(()->power.compressorOff(), power)
    );

    new Button(()->right.getPOV()==0).whenPressed(scoopUp);
    new Button(()->right.getPOV()==180).whenPressed(scoopDown);

    new Button(()->left.getRawButton(1))
      .whenPressed(()->{drivetrain.setMaxOutput(1);})
      .whenReleased(()->{drivetrain.setMaxOutput(.6);});
    
    new Button(()->right.getRawButton(1))
      .whenPressed(()->{drivetrain.setMaxOutput(.3);})
      .whenReleased(()->{drivetrain.setMaxOutput(.6);});
    
    new Button(()->left.getRawButton(2)).whenPressed(
      new ParallelCommandGroup(
        new RunCommand(()->{
            indexer.intakeForward();
            indexer.beltForward();
          },indexer
        ),new RunCommand(
          ()->{
            shooters.intake();
          },shooters
        )
      )
      
    ).whenReleased(
      new ParallelCommandGroup(
        new RunCommand(()->{
            indexer.beltSTOP();;
            indexer.intakeSTOP();
          },indexer
        ),new RunCommand(
          ()->{
            shooters.STOP();
          },shooters
        )
      )
    );
    new Button(()->left.getRawButton(3)).whenPressed(
      new ParallelCommandGroup(
        new InstantCommand(()->shooters.setVolts(-MotorControllerValues.kShooterIntakeValue), shooters),
        new InstantCommand(()->indexer.VOMIT(), indexer)
      )
    ).whenReleased(
      new ParallelCommandGroup(
        new InstantCommand(()->shooters.STOP(), shooters),
        new InstantCommand(()->{indexer.beltSTOP();indexer.intakeSTOP();}, indexer)
      )    
    );
    //Network
    //Left Stick
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
    return taxiBack(1.5, -.225)
      .andThen(scoopUp)
      .andThen(new InstantCommand(
        ()->{shooters.shoot(-7.5);}, shooters))
      .andThen(new WaitCommand(2))
      .andThen(new RunCommand(()->indexer.beltReverse(), indexer).withTimeout(2))
      .andThen(new InstantCommand(()->indexer.beltSTOP(),indexer))
      .andThen(new InstantCommand(()->shooters.STOP(),shooters));
      
    //return null;
  }
}
