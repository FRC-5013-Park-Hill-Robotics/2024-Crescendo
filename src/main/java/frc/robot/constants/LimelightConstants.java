package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LimelightConstants {
    public static final double HORIZONTAL_OFFSET = 0;
    public static final double VERTICAL_OFFSET = 0;
    public static final double ALLIGNMENT_TOLLERANCE_RADIANS = Units.degreesToRadians(3);
    public static final double CONE_WIDTH = 237;
    public static final double CONE_OFFSET = 20;
    public static final int POSE_ESTIMATION = 0;
    public static final int APRIL_TAG_TARGETING = 0;
    public static final int GAME_PIECE_RECOGNITION = 1;

    public static final int APRIL_TAG_BLUE_SPEAKER = 2;
    public static final int APRIL_TAG_RED_SPEAKER = 3;

    public static final double FRONT_LL_VERTICAL_ANGLE_DEG = 23.5;

    //FRONT DISTANCE FROM CENTER OF BOT IN THE X DIMENSION
    public static final double FRONT_LL_X_OFFSET_INCHES = 7.5;

    //FRONT DISTAMCE FROM CENTER OF BOT IN THE Y DIMENSION
    public static final double FRONT_LL_Y_OFFSET_INCHES = 8;

    //FRONT HEIGHT FROM FLOOR
    public static final double FRONT_LL_HEIGHT_OFFSET_INCHES = 8;



    //BACK HEIGHT FROM FLOOR
    public static final double BACK_LL_HEIGHT_OFFSET_INCHES = 12.263;
    public static final Double GETSPEAKERSKEW(){
      Double skew = -1.5;
      return skew;
    }



}
