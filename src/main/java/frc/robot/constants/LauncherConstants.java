// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.trobot5013lib.LinearInterpolator;

/** Add your docs here. */
public final class LauncherConstants {
    public final static int LAUNCHER_SHOULDER_MOTOR_CAN_ID = CANConstants.LAUNCHER_SHOULDER_MOTOR_CAN_ID;
    public final static int ENCODER_DIO_PORT = 0;
    public final static double OFFSET_RADIANS = Math.toRadians(257);
    public final static int LAUNCHER_TOP_CAN_ID = CANConstants.LAUNCHER_TOP_CAN_ID;
    public final static int LAUNCHER_BOTTOM_CAN_ID = CANConstants.LAUNCHER_BOTTOM_CAN_ID;
    public static final class RotationGains {
        public static final Rotation2d kPositionTolerance= Rotation2d.fromDegrees(1);
        public static final double kP = 7.5013;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
        public static final double kS = 0.6;
        public static final double kG = 0.3;
        public static final double kGHhigh = 0.3;
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
    public static final class TargetConstants{
        //ty is the first number, angle in degrees is the second
		public static final double[][] TY_ANGLE_ARRAY = {
            //subwoofer angle is 63 with a speed of 55
				

				{15,53},
				{5,47},
				{-1,41},
				{-4.71, 38.5},
                {-6,37},
				{-6.8,35},
				{-8.6,34},
				{-10,32}
			};
        //ty is the first number, shooter speed in rps is the second
		public static final double[][] TY_SHOOTER_SPEED_ARRAY = {
			{15,50},
			{5,50},
			{-1,55},
			{-4.71,55},
			{-6.8,55},
			{-8.6,55},
			{-10,60},
		};
				

		public static final LinearInterpolator LAUNCHER_TY_ANGLE_INTERPOLATOR = new LinearInterpolator(TY_ANGLE_ARRAY);
		public static final LinearInterpolator LAUNCHER_TY_SHOOTER_SPEED_INTERPOLATOR = new LinearInterpolator(TY_SHOOTER_SPEED_ARRAY);
    }
    public final static double RETRACT_SETPOINT = 0;
    public static final double AMP_ANGLE_RADANS = Math.toRadians(98);//Math.toRadians(96.5);
    public static final double DUCK_RADIANS = Math.toRadians(31);
    public static final double START_ANGLE_RADIANS = Math.toRadians(60);

    public static final double SPEAKER_ANGLE_RADIANS = Math.toRadians(61);
    public static final double PODIUM_ANGLE_RADIANS = Math.toRadians(43);

    public static final double SHOULDER_ANGLE_MAX = Math.toRadians(65);
    public static final double SHOULDER_ANGLE_MIN = Math.toRadians(30);

}
