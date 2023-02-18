// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ArmCommand;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.ArmSubsystem;

import frc.robot.commands.GripperCommand;
import frc.robot.commands.WristCommand;




/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private Command m_autoCommand;

  private ArmCommand m_armCommand;
  private ArmCommand armUpCommand;
  private ArmCommand armDownCommand;

  public final GenericHID operator;
  public final CommandXboxController controller;

  private final ArmSubsystem armSubsystem;
  private GripperSubsystem gripperSubsystem;
  public DriveSubsystem driveSubsystem;
  private GyroSubsystem gyroSubsystem;


  private GripperCommand gripperCommand;
  private WristCommand wristCommand;
  public DriveCommand driveCommand;

  public JoystickButton gripperToggle; // MAY WANT THIS TO BE GRIPPER TOGGLE/WHEN HELD
  public JoystickButton gripperSafety;

  private JoystickButton armMoveUpBtn;
  private JoystickButton armMoveDownBtn;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    this.armSubsystem = new ArmSubsystem();
    //this.armSubsystem.armMovePosition(2048);
    this.gripperSubsystem = new GripperSubsystem();
    this.gyroSubsystem = new GyroSubsystem();
    this.driveSubsystem = new DriveSubsystem(gyroSubsystem);

    this.operator = new GenericHID(Constants.OPERATOR_CONTROLLER);
    this.controller = new CommandXboxController(Constants.DRIVER_CONTROLLER);

    // Configure the button bindings
    initCommands();
  }

  /** Initialize the commands */
  public void initCommands() {
    // Initiate commands.
    this.gripperCommand = new GripperCommand(gripperSubsystem);
    this.wristCommand = new WristCommand();
    this.m_armCommand = new ArmCommand(armSubsystem, false, 0);

    this.armDownCommand = new ArmCommand(armSubsystem, true, -0.2);
    this.armUpCommand = new ArmCommand(armSubsystem, true, 0.2);
    this.driveCommand = new DriveCommand(controller, gyroSubsystem, driveSubsystem);
  
    this.configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    gripperSafety = new JoystickButton(operator, Constants.GRIPPER_SAFETY);
    gripperToggle = new JoystickButton(operator, Constants.GRIPPER_TOGGLE);
    gripperToggle
          .and(gripperSafety)
                      .toggleOnTrue(this.gripperCommand);

    armMoveUpBtn = new JoystickButton(operator, Constants.ARM_UP_BTN);
    armMoveDownBtn = new JoystickButton(operator, Constants.ARM_DOWN_BTN);

    armMoveDownBtn.onTrue(armDownCommand);
    armMoveUpBtn.onTrue(armUpCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
