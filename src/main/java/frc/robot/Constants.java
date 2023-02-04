// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int shoulderFalconId = 0;
    public static final int wristFalconId = 1;
    
    // Gripper Pneumatics
    public static final PneumaticsModuleType PH = PneumaticsModuleType.REVPH;
    public static final int PH_GRIPPER_CLOSE = 2;
    public static final int PH_GRIPPER_OPEN = 3;
    public static final int PH_CAN_ID = 18;
    // Controller
    public static final int OPERATOR_CONTROLLER = 1;

    // Buttons
    public static final int GRIPPER_TOGGLE = 0; // Talk to operator on if held
    public static final int GRIPPER_SAFETY = 1; 
    public static final int UNDERFINED_BUTTON_3 = 2; // Implement some sort of wrist reset
    public static final int UNDERFINED_BUTTON_4 = 3;
    public static final int UNDERFINED_BUTTON_5 = 4;
    public static final int UNDERFINED_BUTTON_6 = 5;
    public static final int UNDERFINED_BUTTON_7 = 6;
    // add more if needed



}
