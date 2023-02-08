// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private AnalogInput WristAnalogInput;
  private AnalogInput ShoulderAnalogInput;
  private WPI_TalonSRX WristFalcon;
  private WPI_TalonFX ShoulderFalcon;

    public ArmSubsystem() {
    ShoulderAnalogInput = new AnalogInput(Constants.SHOULDER_PE_ID);
    WristAnalogInput = new AnalogInput(Constants.WRIST_PE_ID);
    ShoulderFalcon = new WPI_TalonFX(Constants.SHOULDER_FALCON_ID);
    WristFalcon = new WPI_TalonSRX(Constants.WRIST_FALCON_ID);
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
      // .getvoltage shoulderAnalogInput;
      return 0.0;
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    //int wristPos = WristAnalogInput.getValue();
    //SmartDashboard.putNumber("Wrist angle",wristPos); 
    //double wSpeed = wristPos/4000.0;
    //WristFalcon.set(wSpeed);
    //SmartDashboard.putNumber("Speed ",wSpeed);
        } 

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  public void pid() {
    //error = (|currentPostion - exact location| / exact location) *100
    // kp = distance(from the pmeter) * error 
    //ki = time = PID_i = PID_i + ki * error
    //kd = speed = dx/dt(distance/time)

    //PID_pd= kp*error + kd*(error-pervious_error)/time

    //PID = kp*error + PID_i + Ki*error + kd(error-pervious_error)/time
  }
  
}
