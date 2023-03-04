package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmCommand extends CommandBase {

    private ArmSubsystem armSubsystem;
    private boolean relative;
    private double setAngle;

    public ArmCommand(ArmSubsystem subsystem, boolean relative, double setAngle) {
        this.armSubsystem = subsystem;
        this.relative = relative;
        this.setAngle = setAngle;

        addRequirements(subsystem);
    }

    public void setTargetAngle(double setAngle) {
        this.setAngle = setAngle;
    }

    @Override
    public void execute() {
        if (relative == true) {
            armSubsystem.setArmPoistion(setAngle + armSubsystem.getArmSetpoint());
        } else {
            armSubsystem.setArmPoistion(setAngle);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}