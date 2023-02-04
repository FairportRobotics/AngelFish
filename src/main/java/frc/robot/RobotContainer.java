// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.GripperSubsystem;
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

  public final GenericHID operator;
  private ArmSubsystem armSubsystem;
  private GripperSubsystem gripperSubsystem;


  private GripperCommand gripperCommand;
  private WristCommand wristCommand;

  public JoystickButton gripperToggle; // MAY WANT THIS TO BE GRIPPER TOGGLE/WHEN HELD
  public JoystickButton gripperSafety;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    this.armSubsystem = new ArmSubsystem();
    this.gripperSubsystem = new GripperSubsystem();

    this.operator = new GenericHID(Constants.OPERATOR_CONTROLLER);

    // Configure the button bindings
    initCommands();
  }
  public void initCommands() {
    // Initiate commands.
    this.m_autoCommand = new WristCommand();
    this.gripperCommand = new GripperCommand(gripperSubsystem);
    this.wristCommand = new WristCommand();

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
