package frc.robot;

public class Constants {

    public static final int FRONT_LEFT_DRIVE_ID = 1;
    public static final int FRONT_LEFT_SWERVE_ID = 2;
    public static final int FRONT_LEFT_ENCODER_ID = 3;
    public static final double FRONT_LEFT_SWERVE_OFFSET = -263.203;

    public static final int FRONT_RIGHT_DRIVE_ID = 4;
    public static final int FRONT_RIGHT_SWERVE_ID = 5;
    public static final int FRONT_RIGHT_ENCODER_ID = 6;
    public static final double FRONT_RIGHT_SWERVE_OFFSET = -335.391;


    public static final int BACK_LEFT_DRIVE_ID = 7;
    public static final int BACK_LEFT_SWERVE_ID = 8;
    public static final int BACK_LEFT_ENCODER_ID = 9;
    public static final double BACK_LEFT_SWERVE_OFFSET = 198.896;

    public static final int BACK_RIGHT_DRIVE_ID = 10;
    public static final int BACK_RIGHT_SWERVE_ID = 11;
    public static final int BACK_RIGHT_ENCODER_ID = 12;
    public static final double BACK_RIGHT_SWERVE_OFFSET = -107.402;

    public static final double WHEEL_RADIUS = .04;// in meters
    public static final double DRIVE_RATIO = 8.14;
    public static final double STEERING_RATIO = 7/150;
    public static final double DRIVE_ENCODER_RESOLUTION = 2048;

    public static final double MAX_ANG_VEL = 360;
    public static final double MAX_ANG_ACC = 3600;

    public static final double ENCODER_TICKS_PER_METER = DRIVE_ENCODER_RESOLUTION * DRIVE_RATIO / (2 * Math.PI * WHEEL_RADIUS);

    public static final double GO_TO_POSITION_ERROR = 0.3;
    public static final double GO_TO_VELOCITY_ERROR = 0.1;
}