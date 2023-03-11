// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Util;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {

  private AnalogInput wristAnalogInput;
  private AnalogInput armAnalogInput;

  private AnalogInputSim armAnalogInputSim;
  private AnalogInputSim wristAnalogInputSim;

  private WPI_TalonSRX wristMotor;
  private WPI_TalonFX armFalcon;

  private PIDController wristPIDController;
  private PIDController armPIDController;

  private Solenoid brakeSolenoid;

  private SingleJointedArmSim armSim;
  private SingleJointedArmSim wristSim;

  private Mechanism2d mechanism;
  private MechanismRoot2d root;
  private MechanismLigament2d support;
  private MechanismLigament2d arm;
  private MechanismLigament2d wrist;

  /**
   * Create a new ArmSubsystem.
   */
  public ArmSubsystem(PneumaticHub ph) {
    armAnalogInput = new AnalogInput(Constants.ARM_PE_ID);
    wristAnalogInput = new AnalogInput(Constants.WRIST_PE_ID);

    this.armAnalogInputSim = new AnalogInputSim(armAnalogInput);
    this.wristAnalogInputSim = new AnalogInputSim(wristAnalogInput);

    armFalcon = new WPI_TalonFX(Constants.ARM_FALCON_ID);
    armFalcon.setNeutralMode(NeutralMode.Coast);

    wristMotor = new WPI_TalonSRX(Constants.WRIST_FALCON_ID);
    wristMotor.setInverted(true);
    wristMotor.setNeutralMode(NeutralMode.Coast);

    armPIDController = new PIDController(.005, 0, 0);
    wristPIDController = new PIDController(.0015, 0, 0);

    wristPIDController.setSetpoint(wristAnalogInput.getValue());
    armPIDController.setSetpoint(armAnalogInput.getValue());

    this.brakeSolenoid = ph.makeSolenoid(Constants.ARM_BRAKE_ID);

    this.mechanism = new Mechanism2d(5, 5);
    this.root = mechanism.getRoot("support", 4, 0);
    this.support = root.append(new MechanismLigament2d("support", 2, 90));
    this.arm = support.append(new MechanismLigament2d("arm", 2, 135));
    this.wrist = arm.append(new MechanismLigament2d("wrist", 1, -45));

    this.armSim = new SingleJointedArmSim(
        DCMotor.getFalcon500(1),
        300,
        10,
        1,
        0,
        3,
        1,
        true);

    this.wristSim = new SingleJointedArmSim(
        DCMotor.getFalcon500(1),
        100,
        1,
        0.1,
        -3,
        7,
        1,
        false);

    if (Robot.isSimulation()) {
      armAnalogInputSim.setVoltage(0.3);
      wristAnalogInputSim.setVoltage(0.7);
    }

    this.setName("ArmSubsystem");
  }

  public void brakeOpen() {
    brakeSolenoid.set(false);
  }

  public void brakeClosed() {
    brakeSolenoid.set(true);
  }

  @Override
  public void periodic() {
    double armPower = armPIDController.calculate(armAnalogInput.getValue());
    double wristPower = wristPIDController.calculate(wristAnalogInput.getValue());

    armPower = Math.max(Math.min(armPower, 0.25), -0.25);
    wristPower = Math.max(Math.min(wristPower, 1), -1);

    // If the arm is past its limits, don't allow it to go further
    if (armAnalogInput.getValue() < Constants.ARM_MIN) {
      armPower = Math.max(armPower, 0);
    }
    if (armAnalogInput.getValue() > Constants.ARM_MAX) {
      armPower = Math.min(armPower, 0);
    }
    if (wristAnalogInput.getValue() > Constants.WRIST_MIN) {
      wristPower = Math.min(wristPower, 0);
    }
    if (wristAnalogInput.getValue() < Constants.WRIST_MAX) {
      wristPower = Math.max(wristPower, 0);
    }

    SmartDashboard.putNumber("Arm Position", armAnalogInput.getValue());
    SmartDashboard.putNumber("Arm Power", armPower);
    SmartDashboard.putNumber("Arm SetPoint", armPIDController.getSetpoint());

    SmartDashboard.putNumber("Wrist Setpoint", wristPIDController.getSetpoint());
    SmartDashboard.putNumber("Wrist Power", wristPower);
    SmartDashboard.putNumber("Wrist Potentiometer", wristAnalogInput.getValue());

    SmartDashboard.putData("Arm Widget", mechanism);

    if (atSetPoint()) {
      //setBrakeOn();
    } else {
      //setBrakeOff();
    }

    //if (!brakeSolenoid.get()) {
      armFalcon.set(ControlMode.PercentOutput, armPower);
      wristMotor.set(ControlMode.PercentOutput, wristPower);
    //} else {
      //armFalcon.set(ControlMode.PercentOutput, 0);
      //wristMotor.set(ControlMode.PercentOutput, 0);
    

    // Display the current arm state
    arm.setAngle(Util.map(armAnalogInput.getValue(), Constants.ARM_MIN, Constants.ARM_MAX, 180, 0));
    wrist.setAngle(Util.map(wristAnalogInput.getValue(), Constants.WRIST_MIN, Constants.WRIST_MAX, 90, -90));
  }

  /**
   * Set the target arm position.
   * 
   * @param armAngle ranges from 0 to 4000
   */
  public void setArmPoistion(double armAngle) {
    armPIDController.setSetpoint(armAngle);
    SmartDashboard.putNumber("Arm Setpoint", armAngle);
  }

  public void adjustWristLevel(double offsetAdjustment) {
    // System.out.println(offsetAdjustment);
    // wristOffset += offsetAdjustment;
    wristPIDController.setSetpoint(MathUtil.clamp(offsetAdjustment + wristPIDController.getSetpoint(), Constants.WRIST_MAX, Constants.WRIST_MIN));
  }

  public void adjustArmLevel(double armAdjustment) {
    armPIDController.setSetpoint(MathUtil.clamp(armAdjustment + armPIDController.getSetpoint(), Constants.ARM_MIN, Constants.ARM_MAX));
  }

  public void setBrakeOn() {
    this.brakeSolenoid.set(true);
  }

  public void setBrakeOff() {
    this.brakeSolenoid.set(false);
  }

  public boolean atSetPoint() { // TODO: probably shoud take into account velocity
    return Math.abs(armPIDController.getPositionError()) < Constants.ARM_TOLERANCE;
  }

  /**
   * Get the current arm position.
   * 
   * @return ranges from 0 to 4000
   */
  public double getArmPosition() {
    return armAnalogInput.getValue();
  }

  /**
   * Get the current arm setpoint.
   * 
   * @return current arm setpoint
   */
  public double getArmSetpoint() {
    return armPIDController.getSetpoint();
  }

  /**
   * Get the current wrist position.
   * 
   * @return ranges from 0 to 4000
   */
  public double getWristPosition() {
    return wristAnalogInput.getValue();
  }

  @Override
  public void simulationPeriodic() {
    armSim.setInputVoltage(armFalcon.getMotorOutputVoltage());
    wristSim.setInputVoltage(wristMotor.getMotorOutputVoltage());
    armAnalogInputSim.setVoltage(Util.map(armSim.getAngleRads(), 0, Math.PI, 0.3, 2.3));
    wristAnalogInputSim.setVoltage(Util.map(wristSim.getAngleRads(), -Math.PI / 2, Math.PI / 2, 2.1, 4.8));

    armSim.update(0.02);
    wristSim.update(0.02);
  }
}
