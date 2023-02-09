package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GripperSubsystem extends SubsystemBase{

    private DoubleSolenoid gripperSolenoid;
    private PneumaticHub ph;

    public GripperSubsystem() {
        // Added some stuff from askar to constants and changed the names, PH as well.
        ph = new PneumaticHub(Constants.PH_CAN_ID);
        gripperSolenoid = ph.makeDoubleSolenoid(Constants.PH_GRIPPER_OPEN, Constants.PH_GRIPPER_CLOSE);
        gripperSolenoid.set(Value.kForward);
    }

    // Pistons that push/pull gripper claws -- toggles position
    public void GripperToggle() {
      if (!ph.getPressureSwitch() && ph.getCompressor())
        if(gripperSolenoid.get() == Value.kOff){
          gripperSolenoid.set(Value.kForward);
        }else {
          gripperSolenoid.toggle();
      }
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData(gripperSolenoid);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
