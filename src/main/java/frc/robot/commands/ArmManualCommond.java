package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmManualCommond extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final double speed;

    public ArmManualCommond(ArmSubsystem armSubsystem, double speed){
        this.armSubsystem = armSubsystem;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        armSubsystem.setArmSpeed(speed);
        System.out.println("WE RUNN" + speed);
    }


    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }
}
