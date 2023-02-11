// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private AnalogInput WristAnalogInput;
  private AnalogInput ShoulderAnalogInput;

  private WPI_TalonFX WristFalcon;
  private WPI_TalonFX ShoulderFalcon;

  private PIDController wristPIDController;
  private PIDController ShoulderPIDController;

    public ArmSubsystem() {
       ShoulderAnalogInput = new AnalogInput(Constants.SHOULDER_PE_ID);
        WristAnalogInput = new AnalogInput(Constants.WRIST_PE_ID);

        ShoulderFalcon = new WPI_TalonFX(Constants.SHOULDER_FALCON_ID);
        WristFalcon = new WPI_TalonFX(Constants.WRIST_FALCON_ID);

        ShoulderPIDController = new PIDController(.2, .2,.2);
        wristPIDController = new PIDController(.2, .2,.2);
        this.setName("ArmSubsystem");
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      ShoulderFalcon.set(ControlMode.PercentOutput, ShoulderPIDController.calculate(getArmPosition())/ Constants.Shoulder_SPEED_CONTROL);
      WristFalcon.set(ControlMode.PercentOutput, wristPIDController.calculate(WristAnalogInput.getValue())/ Constants.WRIST_SPEED_CONTROL);
      
      // used for grug 
      int wristPos2 = WristAnalogInput.getValue();
      SmartDashboard.putNumber("Pos ",wristPos2);
      SmartDashboard.putData(ShoulderPIDController);
    }
      
    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }

    public void armMovePosition (double armAngle){   
      ShoulderPIDController.setSetpoint(armAngle);
      wristPIDController.setSetpoint(armAngle - 180);
   }

    public double getArmPosition (){    
    return ShoulderAnalogInput.getValue();
    } 
  
}

