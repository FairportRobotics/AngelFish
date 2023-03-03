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
        if ( this.lightController != null ) {
            this.lightController.fillAll("155060180");
        }
    }
    public void setConeColor(){
        if ( this.lightController != null ) {
        this.lightController.fillAll("235185000");
        }
    }

}