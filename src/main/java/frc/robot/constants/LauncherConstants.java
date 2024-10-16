// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.trobot5013lib.LinearInterpolator;

/** Add your docs here. */
public final class LauncherConstants {
    public final static int LEFT_LAUNCHER_SHOULDER_MOTOR_CAN_ID = CANConstants.LEFT_LAUNCHER_SHOULDER_MOTOR_CAN_ID;
    public final static int RIGHT_LAUNCHER_SHOULDER_MOTOR_CAN_ID = CANConstants.RIGHT_LAUNCHER_SHOULDER_MOTOR_CAN_ID;    public final static int ENCODER_DIO_PORT = 0;
    public final static double OFFSET_RADIANS = Math.toRadians(21);
    public final static int LAUNCHER_TOP_CAN_ID = CANConstants.LAUNCHER_TOP_CAN_ID;
    public final static int LAUNCHER_BOTTOM_CAN_ID = CANConstants.LAUNCHER_BOTTOM_CAN_ID;
    public static final class RotationGains {
        public static final Rotation2d kPositionTolerance= Rotation2d.fromDegrees(1);
        public static final double kP = 28.5013;
        public static final double kI = 0;
        public static final double kD = 0.0;
        public static final double kF = 0;
        public static final double kS = 0;
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
				

				{8.9,26.7},
				{7.6,27.7},
				{6.4,28.5},
                {5.8, 28.5},
				{4.9,29},
                {3.8,31},
                {2.7,33},
				{0.1,35},
				{-6.1,39},
				{-26.5,53}
			};
        //ty is the first number, shooter speed in rps is the second
		public static final double[][] TY_SHOOTER_SPEED_ARRAY = {
				{8.9,50},
				{7.6,50},
				{6.4,50},
				{5.3,50},
                {3.8,50},
                {2.7,50},
				{0.1,50},
				{-6.1,50},
				{-26.5,50}
		};


        //make a skew linear interpolator based on ty
        // public static final double[][] TY_SKEW_ARRAY = {
        //     {}
        // };
				

		public static final LinearInterpolator LAUNCHER_TY_ANGLE_INTERPOLATOR = new LinearInterpolator(TY_ANGLE_ARRAY);
		public static final LinearInterpolator LAUNCHER_TY_SHOOTER_SPEED_INTERPOLATOR = new LinearInterpolator(TY_SHOOTER_SPEED_ARRAY);
        //make a skew linear interpolator based on ty

    }
    public final static double RETRACT_SETPOINT = 0;
    public static final double AMP_ANGLE_RADANS = Math.toRadians(51);//Math.toRadians(96.5);
    public static final double DUCK_RADIANS = Math.toRadians(29);
    public static final double START_ANGLE_RADIANS = Math.toRadians(29);
    //start angle used to be 60, and 51

    public static final double SPEAKER_ANGLE_RADIANS = Math.toRadians(53);
    public static final double PODIUM_ANGLE_RADIANS = Math.toRadians(35);
    public static final double SHUTTLING_ANGLE = Math.toRadians(38);
    public static final double SHOULDER_ANGLE_MAX = Math.toRadians(65);
    public static final double SHOULDER_ANGLE_MIN = Math.toRadians(25);

    public static final double SHOULDER_CURRENT_LIMIT = 4;

    public static final double SHOULDER_CURRENT_THRESHOLD = 0.2;

}
