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
            public static final int kRight1CANID = 1;
            public static final int kRight2CANID = 2;
            public static final int kLeft1CANID = 3;
            public static final int kLeft2CANID = 4;
            public static final int kLifterCANID = 5;
            public static final int kPDHCANID = 6;
            public static final int kPnuematicsHubCANID = 7;
            //CTRE
            public static final int kBeltCANID = 8;
            public static final int kLeftShooterCANID = 9;
            public static final int kRightShooterCANID = 10;
            public static final int kIntakeCANID = 11;
        //PH ports
        public static final int[] scoopSolenoidPort = new int[]{12,13};
        public static final int[] climberSolenoidPort = new int[]{15,14};
        //DIO might be unnessisary
        public static final int[] kLeftShooterEncoderPorts = new int[]{0,1};
        public static final int[] kRightShooterEncoderPorts = new int[]{2,3};
        public static final int kLifterEncoderPort = 4;
        public static final int kBottomLimitPort = 5;
        public static final int kTopLimitPort = 6;
        public static final int kBallSensorPort = 7;
        //Robot USB //no
        //Controller Ports
        public static final int[] kUSBPorts = new int[]{0,1,2,3};
    }
    public static final class AutonTrajectorys{
        public static final Trajectory kAuton1 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0, new Rotation2d(0)), 
            List.of(
                new Translation2d(1, 1),
                new Translation2d(2, -1)
            ), 
            new Pose2d(3, 0, new Rotation2d(0)), 
            AutonCommandConstants.trajectoryConfig
        );
        public static final class BallTrans{
            public static final Translation2d ball1 = new Translation2d(0, 0);
            public static final Translation2d ball2 = new Translation2d(0, 0);
            public static final Translation2d ball3 = new Translation2d(0, 0);
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
        public static final double kWheelDiameter = 6/39.37;
        /**meters */
        public static final double kWheelCircumfrence = Math.PI*kWheelDiameter;
        //encoder stuff
        public static final double kDistancePerRotation = kWheelCircumfrence/10.71; //12.75 gearing
        public static final boolean kRightInverted = true;
        public static final boolean kLeftInverted = false;
        public static final DifferentialDriveKinematics KINEMATICS 
            = new DifferentialDriveKinematics(kTrackWidth);
    }
    public static final class ScoopConstants{
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;
        public static final boolean kLeftMotorInverted = false;
        public static final boolean kRightMotorInverted = false;
        public static final boolean kRollerInversed = true;
        public static final boolean kIntakeInversed = false;
        //METERS
        public static final double kWheelDiameter = 6/39.37;
        public static final int kCPR = 8196;
        //RADIANS
        public static final double kDistancePerPulse = (Math.PI*2)*kWheelDiameter/kCPR;
    }
    public static final class LifterConstants{
        public static final double kP = .63172, kI = 0/*keep 0*/, kD = 0;
        public static final double kS = 0.010889, kV = 4.5699, kA = 1.1728, kG = 0.64846;
        /**radians */
        public static final double kTolerence = .1;
        /**radians*/
        public static final double kSecondClimbingPosition = 0;
        /**radians */
        public static final double kClimbPosition = 0;
        /**radians */
        public static final double kOffSet = -.57;
        /**radians */
        public static final double kIntakeClerence = 0;
        /**radians/sec */
        public static final double kMaxVelocity = 1;
        /**radians/sec^2 */
        public static final double kMaxAcceleration = .64;
        /**meters */
        public static final double kClosestDistance = 0;
        /**meters */
        public static final double kMaxDistance = 0;
        public static final boolean kInverted = false;
    }
    public static final class MotorControllerValues{
        public static final double kIntakeValue = .9;
        public static final double kShooterIntakeValue = .55*12;
        /**meters per second */
        public static final double kShooterVelocity = 7.75;
    }
}
