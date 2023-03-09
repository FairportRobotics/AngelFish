// Open Source Software; you can modify and/or share it under the terms of
// Copyright (c) FIRST and other WPILib contributors.
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ArmCommand extends CommandBase {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_subsystem;
  private final boolean relative;
  public double setAngle;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmCommand(ArmSubsystem subsystem, boolean relative, double setAngle) {
    m_subsystem = subsystem;
    this.relative = relative;
    this.setAngle = setAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    //if boolean is false then armpos=setangle 
    //if boolean is true then arpos=setangle+currentangle
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    if (relative == true) {
      m_subsystem.setArmPoistion(setAngle + m_subsystem.getArmSetpoint());
    } else {
      m_subsystem.setArmPoistion(setAngle);
    }
    System.out.println("WEE");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.atSetPoint();
  }

  
}