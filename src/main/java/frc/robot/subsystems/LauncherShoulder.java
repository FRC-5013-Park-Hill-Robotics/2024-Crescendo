// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.trobot5013lib.HeliumEncoderWrapper;

public class LauncherShoulder extends SubsystemBase {
    private final TalonFX launcherShoulderMotor = new TalonFX(IntakeConstants.INTAKE_WRIST_MOTOR_CAN_ID);
    private final HeliumEncoderWrapper encoder = new HeliumEncoderWrapper(IntakeConstants.INTAKE_ENCODER_CAN_ID);

  /** Creates a new LauncherShoulder. */
  public LauncherShoulder() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getShoulderAngleRadians() {
    return encoder.getAbsPositionRadians();
  }

}
