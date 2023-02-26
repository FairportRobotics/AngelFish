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
        previousTime = System.currentTimeMillis();
        armSubsystem.adjustWristLevel(-delta*deadband(operator.getLeftY(), 0.1)/2);
    }

    private static double deadband(double value, double deadband) {
        if(Math.abs(value) > deadband) {
            if(value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }
    
}
