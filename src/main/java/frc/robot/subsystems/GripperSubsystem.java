package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GripperSubsystem extends SubsystemBase{

    private DoubleSolenoid intakeArmSolenoid;

    public GripperSubsystem() {
        // Added some stuff from askar to constants and changed the names, PCM as well.
        intakeArmSolenoid = new DoubleSolenoid(Constants.PCM, Constants.PCM_GRIPPER_CLOSE, Constants.PCM_GRIPPER_OPEN);
    }

    // Pistons that push/pull gripper claws - need to update for 2023, ripped code from askar
    // dont know what kForward or kReverse are.
    public void GripperClose() {intakeArmSolenoid.set(DoubleSolenoid.Value.kForward);}
    public void GripperOpen() {intakeArmSolenoid.set(DoubleSolenoid.Value.kReverse);}
}
