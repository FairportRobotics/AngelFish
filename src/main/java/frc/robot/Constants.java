package frc.robot;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public final class Constants {

    public static final int ARM_FALCON_ID = 58;
    public static final int WRIST_FALCON_ID = 20;

    public static final int ARM_PE_ID = 1;
    public static final int WRIST_PE_ID = 0;

    public static final int  WRIST_SPEED_CONTROL = 4000;
    public static final int  ARM_SPEED_CONTROL = 4000;

    public static final int WRIST_MIN = 1800;
    public static final int WRIST_MAX = 4000;

    public static final int NEUTRAL_WRIST_OFFSET = 3750;

    public static final int ARM_MIN = 249;
    public static final int ARM_MAX = 1886;

    public static final int ARM_UP_BTN = 4;
    public static final int ARM_DOWN_BTN = 1;

    public static final double SUBSTATION_ANGLE = 1000;
    
    public static final double CONE_LOW_ANGLE = 325;
    public static final double CONE_MID_ANGLE = 1380;
    public static final double CONE_HIGH_ANGLE = 1610;

    public static final double CUBE_LOW_ANGLE = 325;
    public static final double CUBE_MID_ANGLE = 1050;
    public static final double CUBE_HIGH_ANGLE = 1370;
        // Not final id's
    // TODO mesaure wheel base and recalculate offsets
    public static final double WHEEL_BASE = 20;
    public static final double TRACK_WIDTH = 20;

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

    public static final double WHEEL_RADIUS = .04;// in meters
    public static final double DRIVE_RATIO = 8.14;
    public static final double STEERING_RATIO = 7/150;
    public static final double DRIVE_ENCODER_RESOLUTION = 2048;

    public static final double DEADBAND_TRANSLATE = 0.05;
    public static final double DEADBAND_ROTATE = 0.05;

    public static final double GO_TO_POSITION_PID_P = 1;
    public static final double GO_TO_POSITION_PID_I = 0;
    public static final double GO_TO_POSITION_PID_D = 0.5;

    public static final double GO_TO_POSITION_MAX_VELOCITY = 1;
    public static final double GO_TO_POSITION_MAX_ACCELERATION = 0.5;

    public static final double MAX_ANG_VEL = 360;
    public static final double MAX_ANG_ACC = 3600;

    public static final double MAX_SPEED = 3;
    public static final double SLOW_SPEED = 2;

    public static final double ENCODER_TICKS_PER_METER = DRIVE_ENCODER_RESOLUTION * DRIVE_RATIO / (2 * Math.PI * WHEEL_RADIUS);

    public static final double GO_TO_POSITION_ERROR = 0.3;
    public static final double GO_TO_VELOCITY_ERROR = 0.1;

    public static final double SWERVE_DRIVE_P = 0.3;
    public static final double SWERVE_DRIVE_I = 0.0;
    public static final double SWERVE_DRIVE_D = 0.0;

    public static final double SWERVE_STEER_P = 0.2;
    public static final double SWERVE_STEER_I = 0.0;
    public static final double SWERVE_STEER_D = 0.0;

    public static final int SHOULDER_FALCON_ID = 0;
    public static final int SHOULDER_PE_ID = 3;
    // Not final id's
    
    // Gripper Pneumatics
    public static final PneumaticsModuleType PH = PneumaticsModuleType.REVPH;
    public static final int PH_GRIPPER_CLOSE = 2;
    public static final int PH_GRIPPER_OPEN = 0;
    public static final int PH_CAN_ID = 18;
    public static final int PH_BRAKE_CLOSE = 29852;
    public static final int PH_BRAKE_OPEN = 98743;
    // not final ID's for brake
    // Controller
    public static final int DRIVER_CONTROLLER = 0;
    public static final int OPERATOR_CONTROLLER = 1;

    // Buttons
    public static final int GRIPPER_TOGGLE = 2; // Talk to operator on if held
    public static final int GRIPPER_SAFETY = 1; 
    public static final int WRIST_RESET = 3; // Implement some sort of wrist reset
    public static final int UNDERFINED_BUTTON_4 = 4;
    public static final int UNDERFINED_BUTTON_5 = 5;
    public static final int UNDERFINED_BUTTON_6 = 6;
    public static final int UNDERFINED_BUTTON_7 = 7;
}
