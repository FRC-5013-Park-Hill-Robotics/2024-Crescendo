// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.sql.Driver;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  AprilTagFieldLayout aprilTagFieldLayout;

  /** Creates a new Limelight. */
  public Limelight() {}
  
  public void initialize(){
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if(alliance.isPresent()){
        if (aprilTagFieldLayout == null) {
            aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        }
        if (alliance.get() == Alliance.Red) {
            aprilTagFieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
        } 
        else {
            aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        } 
        //m_leftPoseEstimator =new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, m_leftCamera,LeftCamera.robotToCam );
        //m_rightPoseEstimator =new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, m_rightCamera,RightCamera.robotToCam );

        //isinitialized = true;
    }
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

