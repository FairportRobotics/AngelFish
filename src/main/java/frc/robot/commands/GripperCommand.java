package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.Main;

public class GripperCommand extends CommandBase{
    private GripperSubsystem gripperSubsystem;
    
    public GripperCommand(GripperSubsystem gripperSubsystem) {
        this.gripperSubsystem = gripperSubsystem;
    }
    @Override
    public void initialize() {
        gripperSubsystem.GripperToggle();
    }
    @Override
    public boolean isFinished() {
    return true;
    }
}
