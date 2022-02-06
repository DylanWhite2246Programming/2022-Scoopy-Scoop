// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double kG = 9.8066;
    public static double deadzone = .025;
    public static double joystickTune(double x){
        return Math.abs(x)>deadzone ? Math.signum(x)*x*x : 0;
    }
    public static final class Ports{
        //CANID
        //REV
        public static final int kPnuematicsHubCANID = 0;
        public static final int kLeft1CANID = 1;
        public static final int kLeft2CANID = 2;
        public static final int kRight1CANID = 3;
        public static final int kRight2CANID = 4;
        public static final int kLifterCANID = 5;
        public static final int kPDHCANID = 6;
        //CTRE
        public static final int kRollerCANID = 7;
        public static final int kLeftShooterCANID = 8;
        public static final int kRightShooterCANID = 9;
        public static final int kIntakeCANID = 10;
        //PCM ports
        public static final int[] climber1Ports = new int[]{1,2};
        public static final int[] climber2Ports = new int[]{3,4};
        //DIO might be unnessisary
        public static final int[] kLeftShooterEncoderPorts = new int[]{0,1};
        public static final int[] kRightShooterEncoderPorts = new int[]{2,3};
        public static final int kBottomLimitPort = 4;
        public static final int kTopLimitPort = 5;
        public static final int kEntrySensor = 6;
        public static final int kFirstBallSensor = 7;
        public static final int kSecondBallSensor = 8;
        //Robot USB
        //Controller Ports
        public static final int[] kUSBPorts = new int[]{0,1,2,3};
    }
    public static final class AutonTrajectorys{
        public static final Pose2d climbPosition = new Pose2d(0, 0, new Rotation2d(0));
        public static final Trajectory kAuton1 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0, new Rotation2d(0)), 
            List.of(
                new Translation2d(1, 1),
                new Translation2d(2, -1)
            ), 
            new Pose2d(3, 0, new Rotation2d(0)), 
            AutonCommandConstants.trajectoryConfig
        );

    }
    public static final class FieldConstants{
       public static final class StartingPoses{
           //TODO change
           public static final Pose2d red1  = new Pose2d(0, 0, new Rotation2d(0));
           public static final Pose2d red2  = new Pose2d(0, 0, new Rotation2d(0));
           public static final Pose2d red3  = new Pose2d(0, 0, new Rotation2d(0));
           public static final Pose2d blue1 = new Pose2d(0, 0, new Rotation2d(0));
           public static final Pose2d blue2 = new Pose2d(0, 0, new Rotation2d(0));
           public static final Pose2d blue3 = new Pose2d(0, 0, new Rotation2d(0));
       } 
       public static final class Balls{
           public static final Translation2d red1  = new Translation2d(0, 0);
           public static final Translation2d red2  = new Translation2d(0, 0);
           public static final Translation2d red3  = new Translation2d(0, 0);
           public static final Translation2d red4  = new Translation2d(0, 0);
           public static final Translation2d red5  = new Translation2d(0, 0);
           public static final Translation2d red6  = new Translation2d(0, 0);
           public static final Translation2d blue1 = new Translation2d(0, 0);
           public static final Translation2d blue2 = new Translation2d(0, 0);
           public static final Translation2d blue3 = new Translation2d(0, 0);
           public static final Translation2d blue4 = new Translation2d(0, 0);
           public static final Translation2d blue5 = new Translation2d(0, 0);
           public static final Translation2d blue6 = new Translation2d(0, 0);
       }
    }
    public static final class AutonCommandConstants{
        public static final double kMaxVoltage = 10.0;
        /**meters/s */
        public static final double kMaxSpeed = 2.0;
        /**meters/s^2 */
        public static final double kMaxAcceleration = 1.0;
        

        public static final SimpleMotorFeedforward feedForward
            = new SimpleMotorFeedforward(0, 0, 0);
        public static final PIDController leftController
            = new PIDController(1, 0, .1);
        public static final PIDController rightController
            = new PIDController(1, 0, .1);
        public static final DifferentialDriveVoltageConstraint kVoltageConstraints
            = new DifferentialDriveVoltageConstraint(
                feedForward, 
                DrivetrainConstants.KINEMATICS, 
                kMaxVoltage
            );
        public static final TrajectoryConfig trajectoryConfig 
            = new TrajectoryConfig(
                kMaxSpeed, kMaxAcceleration
            )
            .addConstraint(kVoltageConstraints)
            .setKinematics(DrivetrainConstants.KINEMATICS);
        public static final RamseteController controller
            = new RamseteController(0, 0);
    }
    public static final class DrivetrainConstants{
        //chassis stuff
        public static final double kTrackWidth = 0;
        /**meters */
        public static final double kWheelDiameter = 7.94/39.37;
        /**meters */
        public static final double kWheelCircumfrence = Math.PI*kWheelDiameter;
        //encoder stuff
        public static final double kDistancePerRotation = kWheelCircumfrence/12.75; //12.75 gearing
        public static final boolean kRightInverted = false;
        public static final boolean kLeftInverted = true;
        public static final DifferentialDriveKinematics KINEMATICS 
            = new DifferentialDriveKinematics(kTrackWidth);
    }
    public static final class ScoopConstants{
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = false;
        public static final boolean kLeftMotorInverted = false;
        public static final boolean kRightMotorInverted = false;
        public static final boolean kRollerInversed = false;
        public static final boolean kIntakeInversed = false;
        //METERS
        public static final double kWheelDiameter = 6/39.37;
        public static final int kCPR = 8196;
        //RADIANS
        public static final double kDistancePerPulse = (Math.PI*2)*kWheelDiameter/kCPR;
    }
    public static final class LifterConstants{
        public static final double kP = 0, kI = 0/*keep 0*/, kD = 0;
        public static final double kFinalGearRatio = (double)1/51;
        /**radians */
        public static final double kConversionFactor = 2*Math.PI*kFinalGearRatio;
        /**radians/sec */  
        public static final double kVelConversionFactor = kConversionFactor/60;
        /**radians*/
        public static final double kSecondClimbingPosition = 0;
        /**radians */
        public static final double kClimbPosition = 0;
        /**radians */
        public static final double kOffSet = 0; //10 degrees
        /**radians */
        public static final double kIntakeClerence = 0;
        /**radians/sec */
        public static final double kMaxVelocity = 0;
        /**radians/sec^2 */
        public static final double kMaxAcceleration = 0;
        /**meters */
        public static final double kClosestDistance = 0;
        /**meters */
        public static final double kMaxDistance = 0;
    }
    public static final class MotorControllerValues{
        public static final double kIntakeValue = .3;
        public static final double kRollerValues = .25;
        public static final double kClimbSpeed = .75;
        /**meters per second */
        public static final double kShooterVelocity = 10;
    }
}
