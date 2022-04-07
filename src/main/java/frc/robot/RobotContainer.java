// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import frc.robot.Constants.LifterConstants;
import frc.robot.Constants.MotorControllerValues;
import frc.robot.Constants.Ports;
import frc.robot.commands.AlignToBall;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.PowerAndPneumatics;
import frc.robot.subsystems.Shooters;
import frc.robot.subsystems.Vision;
import frc.robot.team2246.Drivestation;
import frc.robot.team2246.NetworktableHandeler;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
  private final SlewRateLimiter leftlimiter = new SlewRateLimiter(.75);
  private final SlewRateLimiter rightlimiter = new SlewRateLimiter(1.2);
  
  // The robot's subsystems and commands are defined here...
  private final Climber climber = new Climber();
  private final Drivetrain drivetrain = new Drivetrain();
  private final Indexer indexer = new Indexer();
  private final PowerAndPneumatics power = new PowerAndPneumatics();
  private final Shooters shooters = new Shooters();
  private final Vision vision = new Vision();

  
  //private final Drivestation controller = new Drivestation(Ports.kUSBPorts);
  private final NetworktableHandeler tableButtons = new NetworktableHandeler();

  private final Joystick left = new Joystick(0);
  private final Joystick right = new Joystick(1);
  private final Joystick bb = new Joystick(2);

  private final BooleanSupplier shooterReady = ()->{return shooters.atSetpoint();};
  //private final ConditionalCommand autoFeedShooters = new ConditionalCommand(belt.reverse, belt.stop, shooterReady);
  //private final ParallelCommandGroup intakeBoth = new ParallelCommandGroup(intake.forward, belt.forward);

  private final InstantCommand startShooter = new InstantCommand(()->{shooters.shoot(MotorControllerValues.kShooterVelocity);}, shooters);
  private final InstantCommand stopShooter = new InstantCommand(()->{shooters.STOP();}, shooters);

  private final InstantCommand extendArm = new InstantCommand(()->{climber.extendBackSolenoid();}, climber);
  private final InstantCommand retrackArm = new InstantCommand(()->{climber.retrackBackSolenoid();}, climber);

  private final InstantCommand extendLiftArm = new InstantCommand(()->{climber.extendLifterSolenoid();}, climber);
  private final InstantCommand retrackLiftArm = new InstantCommand(()->{climber.retrackLifterSolenoid();}, climber);

  SequentialCommandGroup taxiBack(double time, double speed){
    return new SequentialCommandGroup(
      new RunCommand(()->drivetrain.computerDrive(speed, 0), drivetrain).withTimeout(time),
      new InstantCommand(()->{drivetrain.STOP();drivetrain.brake();}, drivetrain),
      new WaitCommand(1),
      new InstantCommand(()->drivetrain.idle(), drivetrain)
    );
  }
  //TODO add magnatude supplier
  //private final AlignToBall alignToBall = new AlignToBall(drivetrain, ()->0, vision.getResults().getBestTarget()::getYaw);

  //TODO change
  //private final InstantCommand startLifter = new InstantCommand(()->{lift.aim(0);}, lift);

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
    //lift.setDefaultCommand(new InstantCommand(()->{lift.setAngle(LifterConstants.kOffSet, false);}, lift));
    //shooters.setDefaultCommand(new InstantCommand(()->{shooters.STOP();}, shooters));
    //vision.setDefaultCommand(
    //  new RunCommand(()->{vision.setOveride(tableButtons.getOverideAutoPipe(), tableButtons.getManualPipe());}, vision)
    //);
    //lift.setDefaultCommand(
    //  //new RunCommand(()->{lift.setMotorVoltage(12*ctrl.getLeftY());}, lift)
    //  //new RunCommand(()->{lift.setGoal(.7);lift.enable();}, lift)
    //);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new Button(()->bb.getRawButton(1)).whenPressed(
      new InstantCommand(()->indexer.intakeForward(), indexer)
    ).whenReleased(
      new InstantCommand(()->indexer.intakeSTOP(), indexer)
    );

    new Button(()->bb.getRawButton(12)).whileHeld(
      new InstantCommand(()->power.compressorOn(), power)
    ).whenReleased(
      new InstantCommand(()->power.compressorOff(), power)
    );

    new Button(()->bb.getRawButton(7)).whenPressed(
      //new InstantCommand(()->shooters.setVolts(-7.), shooters)
      //Kentwood -8.5
      new InstantCommand(()->shooters.shoot(-7), shooters)
    ).whenReleased(
      new InstantCommand(()->shooters.STOP(), shooters)
    );

    new Button(()->bb.getRawButton(6)).whenPressed(
      new RunCommand(()->indexer.beltReverse(), indexer)
    ).whenReleased(
      new InstantCommand(()->indexer.beltSTOP(), indexer)
    );


    new Button(()->bb.getRawButton(4)).whenPressed(extendLiftArm);
    new Button(()->bb.getRawButton(8)).whenPressed(retrackLiftArm);


    new Button(()->right.getPOV()==0).whenPressed(extendArm);
    //new Button(()->right.getPOV()==90).whenPressed(extendLiftArm);
    new Button(()->right.getPOV()==180).whenPressed(retrackArm);
    //new Button(()->right.getPOV()==270).whenPressed(retrackLiftArm);

    new Button(()->left.getRawButton(1))
      .whenPressed(()->{drivetrain.setMaxOutput(1);})
      .whenReleased(()->{drivetrain.setMaxOutput(.6);});

    new Button(()->right.getRawButton(1))
      .whenPressed(()->{drivetrain.STOP();});
    
    new Button(()->right.getRawButton(11)).whenPressed(
      ()->{indexer.beltReverse();}
    ).whenReleased(
      ()->{indexer.beltSTOP();}
    );
    new Button(()->right.getRawButton(7)).whenPressed(
      ()->{
        //shooters.shoot(-10);
        shooters.setVolts(12);
      },shooters
    );
    new Button(()->right.getRawButton(9)).whenPressed(
      ()->{
        shooters.STOP();
      },shooters
    );
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
        new InstantCommand(()->shooters.setVolts(-3), shooters),
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
      .andThen(extendArm)
      .andThen(new InstantCommand(
        ()->{shooters.shoot(-7.5);}, shooters))
      .andThen(new WaitCommand(2))
      .andThen(new RunCommand(()->indexer.beltReverse(), indexer).withTimeout(2))
      .andThen(new InstantCommand(()->indexer.beltSTOP(),indexer))
      .andThen(new InstantCommand(()->shooters.STOP(),shooters));
      
    //return null;
  }
  /**
   * s12 - climber safety switch
   * s13 - compressor toggle
   * b00 - intake in
   * b01 - belt in
   * b02 - shooter in
   * b03 - climb up
   * b10 - intake out
   * b11 - belt reverse
   * b12 - shoot
   * b13 - climb down
   * b20 - scoop up
   * b21 - scoop down
   */
}
