package frc.robot;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public final class Constants {

    // Controller
    public static final int DRIVER_CONTROLLER = 0;
    public static final int OPERATOR_CONTROLLER = 1;

    // Arm & Wrist ID's
    public static final int ARM_FALCON_ID = 58;
    public static final int WRIST_FALCON_ID = 20;

    public static final int ARM_BRAKE_ON = -1; //TODO: Find actual values
    public static final int ARM_BRAKE_OFF = -1;

    public static final int ARM_TOLERANCE = 50;

    public static final int ARM_PE_ID = 1;
    public static final int WRIST_PE_ID = 0;

    // TODO: Remeasure for new arm
    // Arm & Wrist Constraints
    public static final int WRIST_MIN = 1800;
    public static final int WRIST_MAX = 4000;

    public static final int ARM_MIN = 249;
    public static final int ARM_MAX = 1886;

    public static final int NEUTRAL_WRIST_OFFSET = 3750;
    
    // Arm Setpoints
    public static final double SUBSTATION_ANGLE = 1000;
    
    public static final double CONE_LOW_ANGLE = 325;
    public static final double CONE_MID_ANGLE = 1380;
    public static final double CONE_HIGH_ANGLE = 1610;

    public static final double CUBE_LOW_ANGLE = 325;
    public static final double CUBE_MID_ANGLE = 1050;
    public static final double CUBE_HIGH_ANGLE = 1370;

    // Swerve
    public static final double FAST_SPEED = 4; // Meters per second
    public static final double SLOW_SPEED = 2; // Meters per second

    public static final double ROTATION_RATE = 4; // Radians per second

    public static final double DEADBAND_TRANSLATE = 0.05;
    public static final double DEADBAND_ROTATE = 0.05;

    public static final double DRIVETRAIN_WHEELBASE_METERS = 1; //TODO
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 1;

    // Swerve ID's and offsets
    public static final int FRONT_LEFT_DRIVE_ID = 1;
    public static final int FRONT_LEFT_SWERVE_ID = 2;
    public static final int FRONT_LEFT_ENCODER_ID = 3;
    public static final double FRONT_LEFT_SWERVE_OFFSET = 104.062;

    public static final int FRONT_RIGHT_DRIVE_ID = 4;
    public static final int FRONT_RIGHT_SWERVE_ID = 5;
    public static final int FRONT_RIGHT_ENCODER_ID = 6;
    public static final double FRONT_RIGHT_SWERVE_OFFSET = -12.393;


    public static final int BACK_LEFT_DRIVE_ID = 7;
    public static final int BACK_LEFT_SWERVE_ID = 8;
    public static final int BACK_LEFT_ENCODER_ID = 9;
    public static final double BACK_LEFT_SWERVE_OFFSET = -161.895  ;

    public static final int BACK_RIGHT_DRIVE_ID = 10;
    public static final int BACK_RIGHT_SWERVE_ID = 11;
    public static final int BACK_RIGHT_ENCODER_ID = 12;
    public static final double BACK_RIGHT_SWERVE_OFFSET = 291.533 ;
    
    // Gripper Pneumatics
    public static final int PH_GRIPPER_CLOSE = 2;
    public static final int PH_GRIPPER_OPEN = 0;
    public static final int PH_CAN_ID = 18;
    public static final int PH_BRAKE_CLOSE = 29852;
    public static final int PH_BRAKE_OPEN = 98743;
    // not final ID's for brake
    // Controller
    public static final int DRIVER_CONTROLLER = 0;
    public static final int OPERATOR_CONTROLLER = 1;

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
