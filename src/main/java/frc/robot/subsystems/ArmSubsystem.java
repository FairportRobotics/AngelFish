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

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {

    private AnalogInput wristAnalogInput;
    private AnalogInput armAnalogInput;

    private WPI_TalonFX wristFalcon;
    private WPI_TalonFX armFalcon;

    private PIDController wristPIDController;
    private PIDController armPIDController;

    private double wristOffset;

    private Mechanism2d mechanism;
    private MechanismRoot2d root;
    private MechanismLigament2d support;
    private MechanismLigament2d arm;
    private MechanismLigament2d wrist;

    /**
     * Create a new ArmSubsystem.
     */
    public ArmSubsystem() {
        this.armAnalogInput = new AnalogInput(Constants.ARM_PE_ID);
        this.wristAnalogInput = new AnalogInput(Constants.WRIST_PE_ID);

        this.armFalcon = new WPI_TalonFX(Constants.ARM_FALCON_ID);
        this.armFalcon.setNeutralMode(NeutralMode.Coast);

        this.wristFalcon = new WPI_TalonFX(Constants.WRIST_FALCON_ID);
        this.wristFalcon.setNeutralMode(NeutralMode.Coast);

        this.armPIDController = new PIDController(.001, 0, 0);
        this.wristPIDController = new PIDController(.001, 0, 0);

        this.wristPIDController.setSetpoint(0);
        this.armPIDController.setSetpoint(Constants.CUBE_LOW_ANGLE);

        this.wristOffset = Constants.NEUTRAL_WRIST_OFFSET;

        this.mechanism = new Mechanism2d(5, 5);
        this.root = mechanism.getRoot("support", 4, 0);
        this.support = root.append(new MechanismLigament2d("support", 2, 90));
        this.arm = support.append(new MechanismLigament2d("arm", 2, 135));
        this.wrist = arm.append(new MechanismLigament2d("wrist", 1, -45));

        this.setName("ArmSubsystem");
    }

    @Override
    public void periodic() {

        // Calculate PID Loops
        double armPower = armPIDController.calculate(armAnalogInput.getValue());
        double wristPower = wristPIDController.calculate(wristAnalogInput.getValue() + armAnalogInput.getValue() - wristOffset);

        // Limit Arm & Wrist Power
        armPower = Math.max(Math.min(armPower, 0.50), -0.50);
        wristPower = Math.max(Math.min(wristPower, 0.25), -0.25);

        // If the arm is past its limits, don't allow it to go further
        if (armAnalogInput.getValue() < Constants.ARM_MIN) { armPower = Math.max(armPower, 0); }
        if (armAnalogInput.getValue() > Constants.ARM_MAX) { armPower = Math.min(armPower, 0); }
        if (wristAnalogInput.getValue() < Constants.WRIST_MIN) { wristPower = Math.max(wristPower, 0); }
        if (wristAnalogInput.getValue() > Constants.WRIST_MAX) { wristPower = Math.min(wristPower, 0); }

        SmartDashboard.putNumber("Arm Position", armAnalogInput.getValue());
        SmartDashboard.putNumber("Arm Power", armPower);

        SmartDashboard.putNumber("Wrist Position", wristAnalogInput.getValue() + armAnalogInput.getValue() - wristOffset);
        SmartDashboard.putNumber("Wrist Power", wristPower);
        SmartDashboard.putNumber("Wrist Potentiometer", wristAnalogInput.getValue());
        SmartDashboard.putNumber("Wrist Offset", wristOffset);

        SmartDashboard.putData("Arm Widget", mechanism);

        // Set the motor powers
        armFalcon.set(ControlMode.PercentOutput, armPower);
        wristFalcon.set(ControlMode.PercentOutput, wristPower);

        // Display the current arm state
        arm.setAngle(((double) armAnalogInput.getValue() - Constants.ARM_MIN) / ((double) Constants.ARM_MAX - (double) Constants.ARM_MIN) * -135 + 180);
        wrist.setAngle(((double) wristAnalogInput.getValue() - Constants.WRIST_MIN) / ((double) Constants.WRIST_MAX - (double) Constants.WRIST_MIN) * 270);
    }

    public void setArmPoistion(double armAngle) {
        armPIDController.setSetpoint(armAngle);
        SmartDashboard.putNumber("Arm Setpoint", armAngle);
    }

    public void adjustWristLevel(double offsetAdjustment) {
        System.out.println(offsetAdjustment);
        wristOffset += offsetAdjustment;
    }

    public double getArmPosition() {
        return armAnalogInput.getValue();
    }

    public double getArmSetpoint() {
        return armPIDController.getSetpoint();
    }

    public double getWristPosition() {
        return wristAnalogInput.getValue();
    }
}
