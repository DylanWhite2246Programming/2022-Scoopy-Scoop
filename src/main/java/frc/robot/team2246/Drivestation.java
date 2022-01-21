// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.team2246;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Button;

public class Drivestation extends SubsystemBase {
  private GenericHID buttonboardA;
  private GenericHID buttonboardB;
  private Joystick leftStick;
  private Joystick rightStick;
  private Button[] 
    buttonBoardButtons, 
    leftStickButtons, 
    rightStickButtons;
  /**
   * A Subsystem that handles the custom drivestation
   * @param ports array of port values in the orded BBA, BBB, LS, RS
   */
  public Drivestation(int[] ports) {
    buttonboardA = new GenericHID(ports[0]);
    buttonboardB = new GenericHID(ports[1]);
    leftStick = new Joystick(ports[2]);
    rightStick = new Joystick(ports[3]);
    buttonBoardButtons = new Button[]{
      new Button(()->buttonboardA.getRawButton(0)),
      new Button(()->buttonboardA.getRawButton(1)),
      new Button(()->buttonboardA.getRawButton(2)),
      new Button(()->buttonboardA.getRawButton(3)),
      new Button(()->buttonboardA.getRawButton(4)),
      new Button(()->buttonboardA.getRawButton(5)),
      new Button(()->buttonboardA.getRawButton(6)),
      new Button(()->buttonboardA.getRawButton(7)),
      new Button(()->buttonboardA.getRawButton(8)),
      new Button(()->buttonboardA.getRawButton(9)),
      new Button(()->buttonboardA.getRawButton(10)),
      new Button(()->buttonboardA.getRawButton(11)),
      new Button(()->buttonboardB.getRawButton(0)),
      new Button(()->buttonboardB.getRawButton(1)),
      new Button(()->buttonboardB.getRawButton(2)),
      new Button(()->buttonboardB.getRawButton(3)),
      new Button(()->buttonboardB.getRawButton(4)),
      new Button(()->buttonboardB.getRawButton(5)),
      new Button(()->buttonboardB.getRawButton(6)),
      new Button(()->buttonboardB.getRawButton(7)),
      new Button(()->buttonboardB.getRawButton(8)),
      new Button(()->buttonboardB.getRawButton(9)),
      new Button(()->buttonboardB.getRawButton(10)),
      new Button(()->buttonboardB.getRawButton(11))
    };
    leftStickButtons = new Button[]{
      new Button(()->leftStick.getRawButton(0)),
      new Button(()->leftStick.getRawButton(1)),
      new Button(()->leftStick.getRawButton(2)),
      new Button(()->leftStick.getRawButton(3)),
      new Button(()->leftStick.getRawButton(4)),
      new Button(()->leftStick.getRawButton(5)),
      new Button(()->leftStick.getRawButton(6)),
      new Button(()->leftStick.getRawButton(7)),
      new Button(()->leftStick.getRawButton(8)),
      new Button(()->leftStick.getRawButton(9)),
      new Button(()->leftStick.getRawButton(10)),
      new Button(()->leftStick.getRawButton(11))
    };
    rightStickButtons = new Button[]{
      new Button(()->rightStick.getRawButton(0)),
      new Button(()->rightStick.getRawButton(1)),
      new Button(()->rightStick.getRawButton(2)),
      new Button(()->rightStick.getRawButton(3)),
      new Button(()->rightStick.getRawButton(4)),
      new Button(()->rightStick.getRawButton(5)),
      new Button(()->rightStick.getRawButton(6)),
      new Button(()->rightStick.getRawButton(7)),
      new Button(()->rightStick.getRawButton(8)),
      new Button(()->rightStick.getRawButton(9)),
      new Button(()->rightStick.getRawButton(10)),
      new Button(()->rightStick.getRawButton(11))
    };
  }

  public Button[] getButtonBoardArray(){return buttonBoardButtons;}
  public Button[] getLeftStickButtons(){return leftStickButtons;}
  public Button[] getRightStickButtons(){return rightStickButtons;}

  private double tune(double x){return Math.signum(x)*x*x;}

  public double getLeftX(){return tune(leftStick.getX());}
  public double getLeftY(){return tune(leftStick.getY());}
  public double getLeftZ(){return tune(leftStick.getZ());}
  public double getLeftSlider(){return leftStick.getThrottle();}
  public int getLeftPov(){return leftStick.getPOV();}
  public boolean leftPovEquals(int x){return getLeftPov()==x;}

  public double getRightX(){return tune(rightStick.getX());}
  public double getRightY(){return tune(rightStick.getY());}
  public double getRightZ(){return tune(rightStick.getZ());}
  public double getRightSlider(){return rightStick.getThrottle();}
  public int getRightPov(){return rightStick.getPOV();}
  public boolean rightPovEquals(int x){return getRightPov()==x;}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
