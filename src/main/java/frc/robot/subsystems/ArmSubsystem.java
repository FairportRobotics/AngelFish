// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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

  private Solenoid brakeSolenoid;

  /**
   * Create a new ArmSubsystem.
   */
  public ArmSubsystem(PneumaticHub ph) {
    armAnalogInput = new AnalogInput(Constants.ARM_PE_ID);
    wristAnalogInput = new AnalogInput(Constants.WRIST_PE_ID);

    armFalcon = new WPI_TalonFX(Constants.ARM_FALCON_ID);
    armFalcon.setNeutralMode(NeutralMode.Coast);

    wristFalcon = new WPI_TalonFX(Constants.WRIST_FALCON_ID);
    wristFalcon.setNeutralMode(NeutralMode.Coast);

    armPIDController = new PIDController(.001, 0, 0);
    wristPIDController = new PIDController(.001, 0, 0);

    wristPIDController.setSetpoint(0);
    armPIDController.setSetpoint(Constants.CUBE_LOW_ANGLE);

    this.wristOffset = Constants.NEUTRAL_WRIST_OFFSET;

    this.brakeSolenoid = ph.makeSolenoid(Constants.ARM_BRAKE_OFF, Constants.ARM_BRAKE_ON);
    setBrakeOn();

    this.setName("ArmSubsystem");
  }
  
  public void brakeOpen () {
  brakeSolenoid.set(true);
  }

  public void brakeClosed () {
  brakeSolenoid.set(false);
  }

  @Override
  public void periodic() {
    double armPower = armPIDController.calculate(armAnalogInput.getValue());
    double wristPower = wristPIDController.calculate(wristAnalogInput.getValue()+armAnalogInput.getValue()-wristOffset);

    armPower = Math.max(Math.min(armPower, 0.75), -0.75);
    wristPower = Math.max(Math.min(wristPower, 0.50), -0.50);

    if(armAnalogInput.getValue() < Constants.ARM_MIN){
      armPower = Math.max(armPower, 0);
      System.out.println("Limit Arm Power Positive");

    }

    if(armAnalogInput.getValue() > Constants.ARM_MAX){
      armPower = Math.min(armPower, 0);
      System.out.println("Limit Arm Power Negative");
    }

    if(wristAnalogInput.getValue() < Constants.WRIST_MIN){
      wristPower = Math.max(wristPower, 0);
      System.out.println("Limit Wrist Power Positive");
    }

    if(wristAnalogInput.getValue() > Constants.WRIST_MAX){
      wristPower = Math.min(wristPower, 0);
      System.out.println("Limit Wrist Power Negative");
    }

    SmartDashboard.putNumber("Arm Position", armAnalogInput.getValue());
    SmartDashboard.putNumber("Arm Power", armPower);

    SmartDashboard.putNumber("Wrist Position", wristAnalogInput.getValue()+armAnalogInput.getValue()-wristOffset);
    SmartDashboard.putNumber("Wrist Power", wristPower);
    SmartDashboard.putNumber("Wrist Potentiometer", wristAnalogInput.getValue());
    SmartDashboard.putNumber("Wrist Offset", wristOffset);
    
    if (atSetPoint()) {
        setBrakeOn();
    } else {
        setBrakeOff();
    }

    if (brakeSolenoid.get() == Value.kForward) {
      armFalcon.set(ControlMode.PercentOutput, armPower);
      wristFalcon.set(ControlMode.PercentOutput, wristPower);
    } else {
      armFalcon.set(ControlMode.PercentOutput, 0);
      wristFalcon.set(ControlMode.PercentOutput, 0);
    }


  }

  /**
   * Set the target arm position.
   * @param armAngle ranges from 0 to 4000
   */
  public void setArmPoistion(double armAngle) {
    armPIDController.setSetpoint(armAngle);
    SmartDashboard.putNumber("Arm Setpoint", armAngle);
  }

  public void adjustWristLevel(double offsetAdjustment) {
    System.out.println(offsetAdjustment);
    wristOffset += offsetAdjustment;
  }

  public void setBrakeOn() {
    this.brakeSolenoid.set(Value.kReverse);
  }

  public void setBrakeOff() {
    this.brakeSolenoid.set(Value.kForward);
  }
  
  public boolean atSetPoint() { //TODO: probably shoud take into account velocity
    return Math.abs(armPIDController.getPositionError()) < Constants.ARM_TOLERANCE;
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
