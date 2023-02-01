// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;



public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private AnalogInput WristAnalogInput;
  private WPI_TalonFX Wristfalcon;
  private String name;
  double speed = .5;
  public void set( double speed ) {
    
  }
  public ArmSubsystem(int falconId, String name) {
     WristAnalogInput = new AnalogInput(0);
     Wristfalcon = new WPI_TalonFX(falconId); 
     this.name = name;
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
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
