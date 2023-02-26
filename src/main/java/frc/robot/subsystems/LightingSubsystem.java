package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SerialPort;

import com.fairportrobotics.frc.poe.controllers.lighting.ArduinoLightingController;

public class LightingSubsystem extends SubsystemBase{
    
    ArduinoLightingController lightController;

    public LightingSubsystem(){
        this.lightController = new ArduinoLightingController(9600, SerialPort.Port.kUSB);
    }

    public void setCubeColor(){
        this.lightController.fillAll("153062181");
    }
    public void setConeColor(){
        this.lightController.fillAll("235184000");
    }

}
