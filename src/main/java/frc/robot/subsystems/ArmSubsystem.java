// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.time.*;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private AnalogInput WristAnalogInput;
  private AnalogInput ShoulderAnalogInput;
  private WPI_TalonSRX WristFalcon;
  private WPI_TalonFX ShoulderFalcon;
  private PIDController wristPIDController;
  private PIDController armPIDController;

    public ArmSubsystem() {
       ShoulderAnalogInput = new AnalogInput(Constants.SHOULDER_PE_ID);
        WristAnalogInput = new AnalogInput(Constants.WRIST_PE_ID);
        ShoulderFalcon = new WPI_TalonFX(Constants.SHOULDER_FALCON_ID);
        WristFalcon = new WPI_TalonSRX(Constants.WRIST_FALCON_ID);
        wristPIDController = new PIDController(.2, .2,.2);
        armPIDController = new PIDController(.2, .2,.2);
     this.setName("ArmSubsystem");
    }
    public void armMovePosition (){
      //  for(int getArmPosition=1; 1<50;i++);
      // need range of voltage/ range of motion of Arm
      // need exact voltage value of 3 or 4 possitons
      // some sort of if statement will be used to
      // move the arm to a new possition
    }
    
    public double getArmPosition (){
      return 0.0;
    }

    public double getwristPosition (){
       return WristAnalogInput.getValue();
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      WristFalcon.set(ControlMode.PercentOutput, wristPIDController.calculate(getwristPosition() / 4000, 10));
    int wristPos2 = WristAnalogInput.getValue();
    SmartDashboard.putNumber("Speed ",wristPos2);
        } 

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
