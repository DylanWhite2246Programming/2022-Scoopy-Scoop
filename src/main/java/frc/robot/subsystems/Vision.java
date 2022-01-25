// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private static PhotonCamera cam = new PhotonCamera("cameraName");
  /** Creates a new Vision. */
  public Vision() {}
  
  /***
   * @param index 0 blue, 1 red
   */
  public void setPipe(int index){
    cam.setPipelineIndex(index);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
