package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GripperSubsystem extends SubsystemBase{

    private DoubleSolenoid gripperSolenoid;
    private PneumaticHub ph;

    public GripperSubsystem() {
        // Added some stuff from askar to constants and changed the names, PH as well.
        ph = new PneumaticHub(1);
        Compressor phCompressor = new Compressor(1, Constants.PH);

        gripperSolenoid = ph.makeDoubleSolenoid(Constants.PH_GRIPPER_OPEN, Constants.PH_GRIPPER_CLOSE);
    }

    // Pistons that push/pull gripper claws -- toggles position
    public void GripperToggle() {gripperSolenoid.toggle();}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
