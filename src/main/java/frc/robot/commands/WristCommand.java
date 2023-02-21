package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;

public class WristCommand extends CommandBase {

    private CommandXboxController operator;
    private ArmSubsystem armSubsystem;

    private long previousTime;

    public WristCommand(CommandXboxController operator, ArmSubsystem armSubsystem){
        this.operator = operator;
        this.armSubsystem = armSubsystem;
        this.previousTime = System.currentTimeMillis();
    }
    @Override
    public void execute() {
        long delta = System.currentTimeMillis() - previousTime;
        armSubsystem.adjustWristOffset(delta*operator.getRightY());
        previousTime = System.currentTimeMillis();
    }
    
}
