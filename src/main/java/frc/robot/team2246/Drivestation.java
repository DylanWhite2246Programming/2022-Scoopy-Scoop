// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.team2246;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Button;

public class Drivestation extends SubsystemBase {
  private static GenericHID buttonboardA;
  private static GenericHID buttonboardB;
  private static Joystick leftStick;
  private static Joystick rightStick;
  
  private SlewRateLimiter limiter = new SlewRateLimiter(.5);
  private double tune(double x){return Math.signum(x)*x*x;}
  /**
   * A Subsystem that handles the custom drivestation
   * @param ports array of port values in the orded BBA, BBB, LS, RS
   */
  public Drivestation(int[] ports) {
    buttonboardA = new GenericHID(ports[0]);
    buttonboardB = new GenericHID(ports[1]);
    leftStick = new Joystick(ports[2]);
    rightStick = new Joystick(ports[3]);
  }

  public boolean climbSafety(){return s03.getAsBoolean();}
  public boolean slewImposer(){return s02.getAsBoolean();}

  //ButtonBoard
  public final Button s00  = new Button(()->buttonboardA.getRawButton(0));
  public final Button s01  = new Button(()->buttonboardA.getRawButton(1));
  public final Button s02  = new Button(()->buttonboardA.getRawButton(2));
  public final Button s03  = new Button(()->buttonboardA.getRawButton(3));
  public final Button s10  = new Button(()->buttonboardA.getRawButton(4));
  public final Button s11  = new Button(()->buttonboardA.getRawButton(5));
  public final Button s12  = new Button(()->buttonboardA.getRawButton(6));
  public final Button s13  = new Button(()->buttonboardA.getRawButton(7));
 
  public final Button b00  = new Button(()->buttonboardB.getRawButton( 0));
  public final Button b01  = new Button(()->buttonboardB.getRawButton( 1));
  public final Button b02  = new Button(()->buttonboardB.getRawButton( 2));
  public final Button b03  = new Button(()->buttonboardB.getRawButton( 3));
  public final Button b10  = new Button(()->buttonboardB.getRawButton( 4));
  public final Button b11  = new Button(()->buttonboardB.getRawButton( 5));
  public final Button b12  = new Button(()->buttonboardB.getRawButton( 6));
  public final Button b13  = new Button(()->buttonboardB.getRawButton( 7));
  public final Button b20  = new Button(()->buttonboardB.getRawButton( 8));
  public final Button b21  = new Button(()->buttonboardB.getRawButton( 9));
  public final Button b22  = new Button(()->buttonboardB.getRawButton(10));
  public final Button b23  = new Button(()->buttonboardB.getRawButton(11));
   
  // left joystick  
  public final Button ls0  = new Button(()->leftStick.getRawButton(0));
  public final Button ls1  = new Button(()->leftStick.getRawButton(1));
  public final Button ls2  = new Button(()->leftStick.getRawButton(2));
  public final Button ls3  = new Button(()->leftStick.getRawButton(3));
  public final Button ls4  = new Button(()->leftStick.getRawButton(4));
  public final Button ls5  = new Button(()->leftStick.getRawButton(5));
  public final Button ls6  = new Button(()->leftStick.getRawButton(6));
  public final Button ls7  = new Button(()->leftStick.getRawButton(7));
  public final Button ls8  = new Button(()->leftStick.getRawButton(8));
  public final Button ls9  = new Button(()->leftStick.getRawButton(9));
  public final Button ls10 = new Button(()->leftStick.getRawButton(10));
  //public final Button ls11 = new Button(()->leftStick.getRawButton(11));

  public double getLeftX(){return tune(leftStick.getX());}
  //public double getLeftY(){return limiter.calculate(tune(leftStick.getY()));}
  public double getLeftY(){
    if(slewImposer()){
      return limiter.calculate(tune(leftStick.getY()));
    }else{
      return tune(leftStick.getY());
    }
  }
  public double getLeftSlider(){return leftStick.getThrottle();}

  //right joystick
  public final Button rs0  = new Button(()->rightStick.getRawButton(0));
  public final Button rs1  = new Button(()->rightStick.getRawButton(1));
  public final Button rs2  = new Button(()->rightStick.getRawButton(2));
  public final Button rs3  = new Button(()->rightStick.getRawButton(3));
  public final Button rs4  = new Button(()->rightStick.getRawButton(4));
  public final Button rs5  = new Button(()->rightStick.getRawButton(5));
  public final Button rs6  = new Button(()->rightStick.getRawButton(6));
  public final Button rs7  = new Button(()->rightStick.getRawButton(7));
  public final Button rs8  = new Button(()->rightStick.getRawButton(8));
  public final Button rs9  = new Button(()->rightStick.getRawButton(9));
  public final Button rs10 = new Button(()->rightStick.getRawButton(10));
  public final Button rs11 = new Button(()->rightStick.getRawButton(11));

  public final Button rsPOVup = new Button(()->getRightPov()==0);
  public final Button rsPOVright = new Button(()->getRightPov()==90);
  public final Button rsPOVdown = new Button(()->getRightPov()==180);
  public final Button rsPOVleft = new Button(()->getRightPov()==270);

  public double getRightX(){return tune(rightStick.getX());}
  //public double getRightY(){return limiter.calculate(tune(rightStick.getY()));}
  public double getRightY(){
    if(slewImposer()){
      return limiter.calculate(tune(rightStick.getY()));
    }else{
      return tune(rightStick.getY());
    }
  }
  public double getRightZ(){return tune(rightStick.getZ());}
  public double getRightSlider(){return rightStick.getThrottle();}
  public int getRightPov(){return rightStick.getPOV();}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

/**
 * 2022 Button Binding
 *s00
 *s01
 *s02
 *s03
 *s10
 *s11
 *s12
 *s13
 *
 *b00
 *b01
 *b02
 *b03
 *b10
 *b11
 *b12
 *b13
 *b20
 *b21
 *b22
 *b23
 *
 *ls0 
 *ls1 
 *ls2 
 *ls3 
 *ls4 
 *ls5 
 *ls6 
 *ls7 
 *ls8 
 *ls9 
 *ls10
 *ls11
 *
 *rs0 
 *rs1 
 *rs2 
 *rs3 
 *rs4 
 *rs5 
 *rs6 
 *rs7 
 *rs8 
 *rs9 
 *rs10
 *rs11
 */