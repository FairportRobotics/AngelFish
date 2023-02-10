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

import java.time.*;

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
    int wristPos2 = WristAnalogInput.getValue();
    //SmartDashboard.putNumber("Wrist angle",wristPos); 
    //double wSpeed = wristPos/4000.0;
    //WristFalcon.set(wSpeed);
    loop();
    SmartDashboard.putNumber("Speed ",wristPos2);
        } 

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  int kd = 10;
  int kp = 2;
  int ki = 3500;
  float wristSetPoint = 21; //where we want the robot 
  float PID_p, PID_i, PID_d, PID_total, time = 0;
  float currentWristpos = 0, distPreviousWristError = 0, wristDistance_error = 0;
  float  period = 50;

  public void pid() {
 

  }
  
  void loop() {

    if (millis() > time+period) //makes internal clock aka refresh rate
    {
      time = millis();    
      currentWristpos = get_dist(100);  // reads analog input  
      System.out.println(millis()); 
      wristDistance_error = wristSetPoint - currentWristpos;   //(where we want to be) - (current pos)
      PID_p = kp * wristDistance_error; 
      float dist_diference = wristDistance_error - distPreviousWristError;     
      PID_d = kd*((wristDistance_error - distPreviousWristError)/period);
        
      if(-3 < wristDistance_error && wristDistance_error < 3)
      {
        PID_i = PID_i + (ki * wristDistance_error);
      }
      else
      {
        PID_i = 0;
      }
    
      PID_total = PID_p + PID_i + PID_d;  
     // PID_total = map(PID_total, -150, 150, 0, 150); dont know 
    
      if(PID_total < 20){PID_total = 20;}
      if(PID_total > 160) {PID_total = 160; } 
    
      WristFalcon.set(PID_total+30);  
      distPreviousWristError = wristDistance_error;
    }
  }
  
  
  
  
  float get_dist(int n)
  {
    long wristPos=0;
    for(int i=0;i<n;i++)
    {
      wristPos = wristPos + WristAnalogInput.getValue();
    }  
     float wSpeed = wristPos/4000;
    //float adc=wristPos/n; not needed
    //float volts = analogRead(adc)*0.0048828125;  // value from sensor * (5/1024)
    //float volts = sum*0.003222656;  // value from sensor * (3.3/1024) EXTERNAL analog refference
  
    //float distance_cm = 17569.7 * pow(adc, -1.2062); not needed
    //float distance_cm = 13*pow(volts, -1); 
    return(wSpeed);
  }

  public long millis()
  {
    Clock clock = Clock.systemDefaultZone();
 
        // get Instant Object of Clock object
        // in milliseconds using millis() method
        long milliseconds = clock.millis();
        return (milliseconds);
  }


}
