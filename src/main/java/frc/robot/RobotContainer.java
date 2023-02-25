// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

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
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.ArmSubsystem;

import frc.robot.commands.GripperOpenCommand;
import frc.robot.commands.WristCommand;

import frc.robot.commands.TimedMoveCommand;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private Command m_autoCommand;

  private ArmCommand armUpCommand;
  private ArmCommand armDownCommand;
  public WristCommand wristCommand;

  public final CommandXboxController controller;
  public final CommandXboxController operator;

  private final ArmSubsystem armSubsystem;
  private GripperSubsystem gripperSubsystem;
  public DriveSubsystem driveSubsystem;

  private GripperOpenCommand openGripperCommand;
  private GripperOpenCommand closeGripperCommand;
  public DriveCommand driveCommand;

  public JoystickButton gripperToggle; // MAY WANT THIS TO BE GRIPPER TOGGLE/WHEN HELD
  public JoystickButton gripperSafety;

  public TimedMoveCommand timedMoveCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    this.armSubsystem = new ArmSubsystem();
    //this.armSubsystem.armMovePosition(2048);
    this.gripperSubsystem = new GripperSubsystem();
    this.driveSubsystem = new DriveSubsystem();

    this.controller = new CommandXboxController(Constants.DRIVER_CONTROLLER);
    this.operator = new CommandXboxController(Constants.OPERATOR_CONTROLLER);

    // Configure the button bindings
    initCommands();
  }

  /** Initialize the commands */
  public void initCommands() {
    // Initiate commands.
    this.openGripperCommand = new GripperOpenCommand(gripperSubsystem, true);
    this.closeGripperCommand = new GripperOpenCommand(gripperSubsystem, false);
    this.driveCommand = new DriveCommand(controller, driveSubsystem);
    this.armDownCommand = new ArmCommand(armSubsystem, true, -50);
    this.armUpCommand = new ArmCommand(armSubsystem, true, 50);
    this.wristCommand = new WristCommand(operator, armSubsystem);
    this.driveCommand = new DriveCommand(controller, driveSubsystem);
    this.timedMoveCommand = new TimedMoveCommand(0.25, 0.25, 30000, driveSubsystem);
  
    this.configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    operator.leftBumper().onTrue(openGripperCommand);
    operator.axisGreaterThan(2, 0.75).onTrue(closeGripperCommand);
    operator.y().onTrue(armUpCommand);
    operator.a().onTrue(armDownCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return driveSubsystem.followTrajectoryCommand(PathPlanner.loadPath("test", new PathConstraints(4, 3)), true);
  }  
}
