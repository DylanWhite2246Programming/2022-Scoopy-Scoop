// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private static PhotonCamera cam = new PhotonCamera("cameraName");
  private boolean overide = false; private int pipe;
  /** Creates a new Vision. */
  public Vision() {
    setPipe();
  }

  public void setOveride(boolean overide, int pipe){
    this.overide=overide;
    this.pipe=pipe;
  }
  
  /***
   * @param index 0 drive mode, 1 blue, 2 red
   */
  public void setPipe(){
    if(overide){
      cam.setPipelineIndex(pipe);
    }else{
      if(DriverStation.getAlliance()==Alliance.Blue){
        cam.setPipelineIndex(1);
      }else if(DriverStation.getAlliance()==Alliance.Red){
        cam.setPipelineIndex(2);
      }else{
        cam.setDriverMode(true);
      }
    }
  }
  public void setDriverMode(boolean value){
    cam.setDriverMode(value);
  }
  public PhotonPipelineResult getResults(){
    return cam.getLatestResult();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
