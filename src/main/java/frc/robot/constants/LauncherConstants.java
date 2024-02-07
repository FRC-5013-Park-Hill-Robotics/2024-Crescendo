// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public final class LauncherConstants {
    public final static int LAUNCHER_SHOULDER_MOTOR_CAN_ID = CANConstants.LAUNCHER_SHOULDER_MOTOR_CAN_ID;
    public final static int LAUNCHER_ENCODER_CAN_ID = CANConstants.LAUNCHER_ENCODER_CAN_ID;
    public final static double OFFSET_RADIANS = Math.PI /2;
    public final static int LAUNCHER_LEFT_CAN_ID = CANConstants.LAUNCHER_LEFT_CAN_ID;
    public final static int LAUNCHER_RIGHT_CAN_ID = CANConstants.LAUNCHER_RIGHT_CAN_ID;
    public static final class RotationGains {
        public static final Rotation2d kPositionTolerance= Rotation2d.fromDegrees(2.5);
        public static final double kP = .5;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
        public static final double kS = 0.1181;
        public static final double kG = 0.4;
        public static final double kV = 3.4;
        public static final double kA = 0.044465;
        public static final double kMaxAcceleration = 0;
        public static final double kMaxSpeed = 0 ;
    }
    public final static class RollerGains {
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
        public static final double kS = 0.1181;
        public static final double kV = 3.4;
        public static final double kA = 0.044465;
    }
    public final static double AMP_SETPOINT = 0;
    public final static double RETRACT_SETPOINT = 0;
}
