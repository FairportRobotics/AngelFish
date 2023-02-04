package frc.robot;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public final class Constants {

    // TODO mesaure wheel base and recalculate offsets
    public static final double WHEEL_BASE = 20;
    public static final double TRACK_WIDTH = 20;

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

    public static final double DEADBAND_TRANSLATE = 0.05;
    public static final double DEADBAND_ROTATE = 0.05;

    public static final double GO_TO_POSITION_PID_P = 1;
    public static final double GO_TO_POSITION_PID_I = 0;
    public static final double GO_TO_POSITION_PID_D = 0.5;

    public static final double GO_TO_POSITION_MAX_VELOCITY = 1;
    public static final double GO_TO_POSITION_MAX_ACCELERATION = 0.5;

    public static final double MAX_ANG_VEL = 360;
    public static final double MAX_ANG_ACC = 3600;

    public static final double ENCODER_TICKS_PER_METER = DRIVE_ENCODER_RESOLUTION * DRIVE_RATIO / (2 * Math.PI * WHEEL_RADIUS);

    public static final double GO_TO_POSITION_ERROR = 0.3;
    public static final double GO_TO_VELOCITY_ERROR = 0.1;

    public static final double SWERVE_DRIVE_P = 0.3;
    public static final double SWERVE_DRIVE_I = 0.0;
    public static final double SWERVE_DRIVE_D = 0.0;

    public static final double SWERVE_STEER_P = 0.2;
    public static final double SWERVE_STEER_I = 0.0;
    public static final double SWERVE_STEER_D = 0.0;
    
    // Gripper Pneumatics
    public static final PneumaticsModuleType PH = PneumaticsModuleType.REVPH;
    public static final int PH_GRIPPER_CLOSE = 2;
    public static final int PH_GRIPPER_OPEN = 3;
    public static final int PH_CAN_ID = 18;
    // Controller
    public static final int DRIVER_CONTROLLER = 0;
    public static final int OPERATOR_CONTROLLER = 1;

    // Buttons
    public static final int GRIPPER_TOGGLE = 0; // Talk to operator on if held
    public static final int GRIPPER_SAFETY = 1; 
    public static final int UNDERFINED_BUTTON_3 = 2; // Implement some sort of wrist reset
    public static final int UNDERFINED_BUTTON_4 = 3;
    public static final int UNDERFINED_BUTTON_5 = 4;
    public static final int UNDERFINED_BUTTON_6 = 5;
    public static final int UNDERFINED_BUTTON_7 = 6;
}
