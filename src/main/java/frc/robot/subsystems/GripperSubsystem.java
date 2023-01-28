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

    // Pistons that push/pull gripper claws -- toggles position
    public void GripperToggle() {intakeArmSolenoid.toggle();}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
