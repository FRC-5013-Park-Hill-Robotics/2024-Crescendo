// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public final class LauncherConstants {
    public final static int LAUNCHER_SHOULDER_MOTOR_CAN_ID = CANConstants.LAUNCHER_SHOULDER_MOTOR_CAN_ID;
    public final static int ENCODER_DIO_PORT = 0;
    public final static double OFFSET_RADIANS = Math.toRadians(257);
    public final static int LAUNCHER_TOP_CAN_ID = CANConstants.LAUNCHER_TOP_CAN_ID;
    public final static int LAUNCHER_BOTTOM_CAN_ID = CANConstants.LAUNCHER_BOTTOM_CAN_ID;
    public static final class RotationGains {
        public static final Rotation2d kPositionTolerance= Rotation2d.fromDegrees(2.5);
        public static final double kP = 7.5013;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
        public static final double kS = 0.6;
        public static final double kG = 0.3;
        public static final double kV = 0.2;
        public static final double kA = 0.1;
        public static final double kMaxSpeed = 1.6 ; //theoretial free speed of the arm is 1.8868
        public static final double kMaxAcceleration = kMaxSpeed * 3;
    }
    public final static class RollerGains {
      /*   public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
        public static final double kS = 0.1181;
        public static final double kV = .34;
        public static final double kA = 0.044465;
        */
        public static final double kP = 0.025371;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
        public static final double kS = 0.39545;
        public static final double kV = 0.12212;
        public static final double kA = 0.0046099;
    }
    public final static double RETRACT_SETPOINT = 0;
    public static final double AMP_ANGLE_RADANS = Math.toRadians(96.5);
    public static final double DUCK_RADIANS = Math.toRadians(50);
    public static final double START_ANGLE_RADIANS = Math.toRadians(60);
}
