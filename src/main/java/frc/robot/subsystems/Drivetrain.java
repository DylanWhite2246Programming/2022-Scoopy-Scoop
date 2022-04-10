// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.Ports;

public class Drivetrain extends SubsystemBase {

  private final CANSparkMax left1 
    = new CANSparkMax(Ports.kLeft1CANID, MotorType.kBrushless);
  private final CANSparkMax left2
    = new CANSparkMax(Ports.kLeft2CANID, MotorType.kBrushless);
  private final CANSparkMax right1 
    = new CANSparkMax(Ports.kRight1CANID, MotorType.kBrushless);
  private final CANSparkMax right2
    = new CANSparkMax(Ports.kRight2CANID, MotorType.kBrushless);

  //private final AHRS NAVX = new AHRS();
  
  private final DifferentialDrive drive 
    = new DifferentialDrive(left1, right1);

  private final DifferentialDriveKinematics kinematics 
    = new DifferentialDriveKinematics(DrivetrainConstants.kTrackWidth);
  
  //private DifferentialDriveOdometry odometry 
  //  = new DifferentialDriveOdometry(
  //    getRotation2d(), 
  //    new Pose2d(0, 0, new Rotation2d(0))
  //  );

  private final PIDController controller 
    = new PIDController(.5, 0, .05);

  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {
    controller.setSetpoint(0);

    left2.follow(left1);
    right2.follow(right1);

    left1.setInverted(DrivetrainConstants.kLeftInverted);
    right1.setInverted(DrivetrainConstants.kRightInverted);

    left1.getEncoder().setPositionConversionFactor(DrivetrainConstants.kDistancePerRotation);
    right1.getEncoder().setPositionConversionFactor(DrivetrainConstants.kDistancePerRotation);

    left1.getEncoder().setVelocityConversionFactor(DrivetrainConstants.kDistancePerRotation/60);
    right1.getEncoder().setVelocityConversionFactor(DrivetrainConstants.kDistancePerRotation/60);

    //NAVX.setAngleAdjustment(0);

    drive.setSafetyEnabled(false);
  }
  public void setSafety(boolean safe){drive.setSafetyEnabled(safe);};
  public void setMaxOutput(double maxOutput){drive.setMaxOutput(maxOutput);}
  public void STOP(){drive.stopMotor();}
  public void brake(){
     left1.setIdleMode(IdleMode.kBrake);
     left2.setIdleMode(IdleMode.kBrake);
    right1.setIdleMode(IdleMode.kBrake);
    right2.setIdleMode(IdleMode.kBrake);
  }
  public void idle(){
     left1.setIdleMode(IdleMode.kCoast);
     left2.setIdleMode(IdleMode.kCoast);
    right1.setIdleMode(IdleMode.kCoast);
    right2.setIdleMode(IdleMode.kCoast);
  }
  
  public void drive(double x, double z){
    //if(z==0){//inputs must be pasted through deadzone filter.
    //  drive.arcadeDrive(x, controller.calculate(getChassisSpeed().omegaRadiansPerSecond), false);
    //}else{drive.arcadeDrive(x, z, false);}
    drive.arcadeDrive(x, z, false);
  }
  public void computerDrive(double x, double z){
    drive.arcadeDrive(x, z, false);
  }

  public void driveVolts(double leftVolts, double rightVolts){
    left1.setVoltage(leftVolts);
    right1.setVoltage(rightVolts);
    drive.feed();
  }

  /**
   * @return the linear velocity in m/s
   */
  private double getLeftWheelLinearVelocity(){return left1.getEncoder().getVelocity();}
  /**
   * @return the linear velocity in m/s
   */
  private double getRightWheelLinearVelocity(){return right1.getEncoder().getVelocity();}

  private double getLeftDistance(){return left1.getEncoder().getPosition();}
  private double getRightDistance(){return right1.getEncoder().getPosition();}

  public double getAverageEncoderDistance(){
    return 
      (left1.getEncoder().getPosition()+right1.getEncoder().getPosition())
        /2.0;}
  public void resetEncoders(){
    left1.getEncoder().setPosition(0);
    right1.getEncoder().setPosition(0);
  }
  
  public ChassisSpeeds getChassisSpeed(){
    return kinematics.toChassisSpeeds(getWheelSpeeds());
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(
      getLeftWheelLinearVelocity(), 
      getRightWheelLinearVelocity()
    );
  }
  
  //public Pose2d getPose(){return odometry.getPoseMeters();}
  //public double getDistanceTo(Translation2d place){
  //  return place.getDistance(getPose().getTranslation());
  //}
  
  //public void resetOdometery(Pose2d pose){
  //  resetEncoders();
  //  odometry.resetPosition(pose, NAVX.getRotation2d());
  //}

  //public void zeroHeading(){NAVX.reset();}
  //public Rotation2d getRotation2d(){return NAVX.getRotation2d();}
  /**Degrees */
  //public double getHeading(){return getRotation2d().getDegrees();}
  /**Radians */
  //public double getHeadingRads(){return NAVX.getRotation2d().getRadians();}
  /**Radians relative only to robot does not include starting pose*/
  //public double getAbsoluteHeading(){return Math.toRadians(NAVX.getAngleAdjustment());}

  //public double getTurnRate(){return -NAVX.getRate();}
  //public double getTurnRateRad(){return Units.degreesToRadians(getTurnRate());}

  public PIDController getTurnController(){return controller;}

  public void feed(){drive.feed();}


  @Override
  public void periodic() {
    //TODO add network table stuff
    //odometry.update(getRotation2d(), getLeftDistance(), getRightDistance());
    SmartDashboard.putNumber("Velocity", getChassisSpeed().vxMetersPerSecond);
    SmartDashboard.putNumber("Rot Speed", getChassisSpeed().omegaRadiansPerSecond);
    //SmartDashboard.putNumber("Heading", getHeadingRads());
    //SmartDashboard.putNumber("Turn Rate", getTurnRateRad());
    SmartDashboard.putNumber("Left Motor Temp", 
      (
       left1.getMotorTemperature()+
       left2.getMotorTemperature()
      )/2
    );
    SmartDashboard.putNumber("Right Motor Temp", 
      (
        right1.getMotorTemperature()+
        right2.getMotorTemperature() 
      )/2
    );
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
