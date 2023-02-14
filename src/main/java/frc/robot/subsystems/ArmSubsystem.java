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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private AnalogInput WristAnalogInput;
  private AnalogInput armAnalogInput;

  private WPI_TalonFX WristFalcon;
  private WPI_TalonFX armFalcon;

  private PIDController wristPIDController;
  private PIDController armPIDController;

    public ArmSubsystem() {
       armAnalogInput = new AnalogInput(Constants.ARM_PE_ID);
        WristAnalogInput = new AnalogInput(Constants.WRIST_PE_ID);

        armFalcon = new WPI_TalonFX(Constants.ARM_FALCON_ID);
        armFalcon.setNeutralMode(NeutralMode.Brake);
        WristFalcon = new WPI_TalonFX(Constants.WRIST_FALCON_ID);
        WristFalcon.setNeutralMode(NeutralMode.Brake);

        armPIDController = new PIDController(.2, .2,.2);
        armPIDController.setSetpoint(armAnalogInput.getValue());
        wristPIDController = new PIDController(.2, .2,.2);
        wristPIDController.setSetpoint(WristAnalogInput.getValue());
        this.setName("ArmSubsystem");
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      armFalcon.set(ControlMode.PercentOutput, armPIDController.calculate(getArmPosition()) / Constants.ARM_SPEED_CONTROL);
     // WristFalcon.set(ControlMode.PercentOutput, wristPIDController.calculate(WristAnalogInput.getValue())/ Constants.WRIST_SPEED_CONTROL);
      
      // used for grug 
      int wristPos2 = WristAnalogInput.getValue();
      SmartDashboard.putNumber("Poswrist ",wristPos2);
      SmartDashboard.putData(armPIDController);

      int armPos2 = armAnalogInput.getValue();
      SmartDashboard.putNumber("Posarm ",armPos2);
      SmartDashboard.putData(armPIDController);

    }
      
    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }

    public void setArmSpeed(double speed )
    {
      //armFalcon.set(ControlMode.PercentOutput, speed);
      WristFalcon.set(ControlMode.PercentOutput, speed);
    }

    public void armMovePosition(double armAngle){

      armPIDController.setSetpoint(armAngle);
   }

   public void wristMovePosition(double wristAngle){
      wristPIDController.setSetpoint(wristAngle - 180);
   }

    public double getArmPosition (){    
    return armAnalogInput.getValue();
    } 
  
}

