package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GripperSubsystem extends SubsystemBase{

    private DoubleSolenoid gripperSolenoid;
    private PneumaticHub ph;
    private final int MIN_RED_CONE = 205;
    private final int MAX_RED_CONE = 265;
    private final int MIN_GREEN_CONE = 154;
    private final int MAX_GREEN_CONE = 214;
    private final int MIN_BLUE_CONE = 0;
    private final int MAX_BLUE_CONE = 30;
    private final int MIN_RED_CUBE = 123;
    private final int MAX_RED_CUBE = 183;
    private final int MIN_GREEN_CUBE = 32;
    private final int MAX_GREEN_CUBE = 92;
    private final int MIN_BLUE_CUBE = 151;
    private final int MAX_BLUE_CUBE = 211;
    private SerialPort lightController;
    private Compressor phCompressor;

    public GripperSubsystem() {
        // Added some stuff from askar to constants and changed the names, PH as well.
        ph = new PneumaticHub(Constants.PH_CAN_ID);
        phCompressor = ph.makeCompressor();
        phCompressor.enableDigital();

        gripperSolenoid = ph.makeDoubleSolenoid(Constants.PH_GRIPPER_OPEN, Constants.PH_GRIPPER_CLOSE);
        gripperSolenoid.set(Value.kForward);
    }

    public GamePiece checkColors(int r, int g,int b)
    {
      if(r >=MIN_RED_CONE && r <= MAX_RED_CONE && g >= MIN_GREEN_CONE && g <= MAX_GREEN_CONE && b >= MIN_BLUE_CONE && b <= MAX_BLUE_CONE)
        return GamePiece.CONE;
      else if(r >= MIN_RED_CUBE && r <= MAX_RED_CUBE && g >= MIN_GREEN_CUBE && g <= MAX_GREEN_CUBE && b >= MIN_BLUE_CUBE && b <= MAX_BLUE_CUBE)
        return GamePiece.CUBE;
      else
        return GamePiece.NONE;
    }

    public enum GamePiece
    {
      NONE,
      CONE,
      CUBE
    }

    // Pistons that push/pull gripper claws -- toggles position
    public void GripperToggle() {
      //if (!ph.getPressureSwitch() && ph.getCompressor())
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
    SmartDashboard.putData(phCompressor);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public int lightingInfo(String lightingInput) {
    return lightController.writeString(lightingInput);
  }
}
