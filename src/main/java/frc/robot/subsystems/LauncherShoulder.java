// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.trobot5013lib.HeliumEncoderWrapper;

public class LauncherShoulder extends SubsystemBase {
  private final TalonFX launcherShoulderMotor = new TalonFX(IntakeConstants.INTAKE_WRIST_MOTOR_CAN_ID);
  private final HeliumEncoderWrapper encoder = new HeliumEncoderWrapper(IntakeConstants.INTAKE_ENCODER_CAN_ID);
  //Create Constraints
  //Create ProfiledPIDController
  //Creagt ArmFeedforward
  //Create TorqueCurrentFOC
 
  /** Creates a new LauncherShoulder. */
  public LauncherShoulder() {
    launcherShoulderMotor.getConfigurator().apply(new TalonFXConfiguration());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //calculate pid based on goal
    //calculate acceleration based on goal
    //calculate feedforward based on setpoints from profile and acceleration
    //set motor to pid + feedforward
  }

  public double getShoulderAngleRadians() {
    return encoder.getAbsPositionRadians();
  }

  // create command to track field pose and interpolated angle based on pose
}
