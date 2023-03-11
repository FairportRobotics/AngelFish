package frc.robot;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public final class Constants {

    // Controller
    public static final int DRIVER_CONTROLLER = 0;
    public static final int OPERATOR_CONTROLLER = 1;

    // Arm & Wrist ID's
    public static final int ARM_FALCON_ID = 20;
    public static final int WRIST_FALCON_ID = 50;

    public static final int ARM_BRAKE_ID = 4; //TODO: Find actual values

    public static final int ARM_TOLERANCE = 50;

    public static final int ARM_PE_ID = 1;
    public static final int WRIST_PE_ID = 0;

    // TODO: Remeasure for new arm
    // Arm & Wrist Constraints
    public static final int WRIST_MIN = 3000;
    public static final int WRIST_MAX = 730;

    public static final int ARM_MIN = 4;
    public static final int ARM_MAX = 1127;
    
    // Arm Setpoints
    public static final double SUBSTATION_ANGLE = 1000;
    
    public static final double CONE_LOW_ANGLE = 325;
    public static final double CONE_MID_ANGLE = 756;
    public static final double CONE_HIGH_ANGLE = 1035;

    public static final double CUBE_LOW_ANGLE = 325;
    public static final double CUBE_MID_ANGLE = 745;
    public static final double CUBE_HIGH_ANGLE = 1100;

    //Wrist Setpoints
    public static final double WRIST_CONE_LOW_ANGLE = 325;
    public static final double WRIST_CONE_MID_ANGLE = 1890;
    public static final double WRIST_CONE_HIGH_ANGLE = 2405;

    public static final double WRIST_CUBE_LOW_ANGLE = 325;
    public static final double WRIST_CUBE_MID_ANGLE = 2000;
    public static final double WRIST_CUBE_HIGH_ANGLE = 2350; 

    // Swerve
    public static final double FAST_SPEED = 2; // Meters per second
    public static final double SLOW_SPEED = 1; // Meters per second

    public static final double ROTATION_RATE = 3; // Radians per second

    public static final double DEADBAND_TRANSLATE = 0.1;
    public static final double DEADBAND_ROTATE = 0.1;

    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5842;
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.4826;

    // Swerve ID's and offsets
    public static final int FRONT_LEFT_DRIVE_ID = 1;
    public static final int FRONT_LEFT_SWERVE_ID = 2;
    public static final int FRONT_LEFT_ENCODER_ID = 3;
    public static final double FRONT_LEFT_SWERVE_OFFSET = -Math.toRadians(282.832);

    public static final int FRONT_RIGHT_DRIVE_ID = 4;
    public static final int FRONT_RIGHT_SWERVE_ID = 5;
    public static final int FRONT_RIGHT_ENCODER_ID = 6;
    public static final double FRONT_RIGHT_SWERVE_OFFSET = -Math.toRadians(241.787);


    public static final int BACK_LEFT_DRIVE_ID = 7;
    public static final int BACK_LEFT_SWERVE_ID = 8;
    public static final int BACK_LEFT_ENCODER_ID = 9;
    public static final double BACK_LEFT_SWERVE_OFFSET = -Math.toRadians(155.918);

    public static final int BACK_RIGHT_DRIVE_ID = 10;
    public static final int BACK_RIGHT_SWERVE_ID = 11;
    public static final int BACK_RIGHT_ENCODER_ID = 12;
    public static final double BACK_RIGHT_SWERVE_OFFSET = -Math.toRadians(107.842);

    // Gripper Pneumatics
    public static final int PH_GRIPPER_CLOSE = 3;
    public static final int PH_GRIPPER_OPEN = 1;
    public static final int PH_CAN_ID = 18;
    // not final ID's for brake
 
    // Cone & Cube RBG Thresholds
    public static final int MIN_RED_CONE = 205;
    public static final int MAX_RED_CONE = 265;
    public static final int MIN_GREEN_CONE = 154;
    public static final int MAX_GREEN_CONE = 214;
    public static final int MIN_BLUE_CONE = 0;
    public static final int MAX_BLUE_CONE = 30;
    public static final int MIN_RED_CUBE = 123;
    public static final int MAX_RED_CUBE = 183;
    public static final int MIN_GREEN_CUBE = 32;
    public static final int MAX_GREEN_CUBE = 92;
    public static final int MIN_BLUE_CUBE = 151;
    public static final int MAX_BLUE_CUBE = 211;
}
