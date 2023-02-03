// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private AnalogInput shoulderAnalogInput;
  double speed = .5;
  public void set( double speed ) {
    
  }
  public ArmSubsystem() {
    shoulderAnalogInput = new AnalogInput(); 
  }

  public void ArmMovePossition (){
    //  for(int getArmPossition=1; 1<50;i++);
    // 
    // need range of voltage/ range of motion of Arm
    // need exact voltage value of 3 or 4 possitons
    // some sort of if statement will be used to 
    // move the arm to a new possition
    //
  }

  public getArmPossition (){
    // .getvoltage shoulderAnalogInput;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
}
