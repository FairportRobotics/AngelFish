// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {

  private AnalogInput wristAnalogInput;
  private AnalogInput armAnalogInput;

  private WPI_TalonFX wristFalcon;
  private WPI_TalonFX armFalcon;

  private PIDController wristPIDController;
  private PIDController armPIDController;

  private double wristOffset;

  /**
   * Create a new ArmSubsystem.
   */
  public ArmSubsystem() {
    armAnalogInput = new AnalogInput(Constants.ARM_PE_ID);
    wristAnalogInput = new AnalogInput(Constants.WRIST_PE_ID);

    armFalcon = new WPI_TalonFX(Constants.ARM_FALCON_ID);
    armFalcon.setNeutralMode(NeutralMode.Brake);

    wristFalcon = new WPI_TalonFX(Constants.WRIST_FALCON_ID);
    wristFalcon.setNeutralMode(NeutralMode.Brake);

    armPIDController = new PIDController(.001, 0, 0);
    wristPIDController = new PIDController(.001, 0, 0);

    wristPIDController.setSetpoint(wristAnalogInput.getValue());
    armPIDController.setSetpoint(armAnalogInput.getValue());

    wristOffset = Constants.WRIST_OFFSET;

    this.setName("ArmSubsystem");
  }

  @Override
  public void periodic() {
    double armPower = armPIDController.calculate(armAnalogInput.getValue());
    double wristPower = wristPIDController.calculate(wristAnalogInput.getValue()+armAnalogInput.getValue()+wristOffset);

    armPower = Math.max(Math.min(armPower, 0.25), -0.25);
    wristPower = Math.max(Math.min(wristPower, 0.25), -0.25);

    SmartDashboard.putNumber("Arm Position", armAnalogInput.getValue());
    SmartDashboard.putNumber("Arm Power", armPower);

    SmartDashboard.putNumber("Wrist Position", wristAnalogInput.getValue());
    SmartDashboard.putNumber("Wrist Power", wristPower);

    armFalcon.set(ControlMode.PercentOutput, armPower);
    wristFalcon.set(ControlMode.PercentOutput, wristPower);
  }

  /**
   * Set the target arm position.
   * @param armAngle ranges from 0 to 4000
   */
  public void setArmPoistion(double armAngle) {
    armPIDController.setSetpoint(armAngle);
    SmartDashboard.putNumber("Arm Setpoint", armAngle);
  }

  /**
   * Set the target wrist position.
   * @param wristAngle ranges from 0 to 4000
   */
  public void adjustWristOffset(double offsetAdjustment) {
    wristOffset = Math.max(Math.min(wristOffset + offsetAdjustment, 2000), 0);
    SmartDashboard.putNumber("Wrist Offset", wristOffset);
  }

  /**
   * Get the current arm position.
   * @return ranges from 0 to 4000
   */
  public double getArmPosition() {
    return armAnalogInput.getValue();
  }

  /**
   * Get the current arm setpoint.
   * @return current arm setpoint
   */
  public double getArmSetpoint() {
    return armPIDController.getSetpoint();
  }
  /**
   * Get the current wrist position.
   * @return ranges from 0 to 4000
   */
  public double getWristPosition() {
    return wristAnalogInput.getValue();
  }
}
