package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;

public class GripperOpenCommand extends CommandBase{

    private GripperSubsystem gripperSubsystem;
    private boolean open;
    
    public GripperOpenCommand(GripperSubsystem gripperSubsystem, boolean open) {
        this.gripperSubsystem = gripperSubsystem;
        this.open = open;
    }
    @Override
    public void execute() {
        if (open) {
            gripperSubsystem.setOpen();
        } else {
            gripperSubsystem.setClosed();
        }
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
