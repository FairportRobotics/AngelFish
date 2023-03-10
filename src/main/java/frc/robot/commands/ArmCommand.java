package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmCommand extends CommandBase {

    private ArmSubsystem armSubsystem;
    private boolean relative;
    private double setAngle;

<<<<<<< HEAD
  @Override
  public void initialize() {
    if (relative == true) {
      m_subsystem.setArmPoistion(setAngle + m_subsystem.getArmSetpoint());
    } else {
      m_subsystem.setArmPoistion(setAngle);
=======
    public ArmCommand(ArmSubsystem subsystem, boolean relative, double setAngle) {
        this.armSubsystem = subsystem;
        this.relative = relative;
        this.setAngle = setAngle;

        addRequirements(subsystem);
>>>>>>> origin/master
    }

    public void setTargetAngle(double setAngle) {
        this.setAngle = setAngle;
    }

<<<<<<< HEAD
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.atSetPoint();
  }
=======
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
>>>>>>> origin/master

}