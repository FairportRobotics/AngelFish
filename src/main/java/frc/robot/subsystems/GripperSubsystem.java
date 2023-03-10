package frc.robot.subsystems;

import com.fairportrobotics.frc.poe.sensors.colorsensors.TCS34725;
import com.fairportrobotics.frc.poe.sensors.colorsensors.TCS34725.TCS34725_RGB;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GripperSubsystem extends SubsystemBase {

  private DoubleSolenoid gripperSolenoid;
  private DoubleSolenoid brakeSolenoid;
  private PneumaticHub ph;
  private TCS34725 colorSensor;
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

  public GripperSubsystem(PneumaticHub ph) {
    gripperSolenoid = ph.makeDoubleSolenoid(Constants.PH_GRIPPER_OPEN, Constants.PH_GRIPPER_CLOSE);
    gripperSolenoid.set(Value.kForward);
    brakeSolenoid = ph.makeDoubleSolenoid(Constants.PH_BRAKE_OPEN, Constants.PH_BRAKE_CLOSE);
    brakeSolenoid.set(Value.kForward);

    colorSensor = new TCS34725();
  }

  public GamePiece checkColors(int r, int g, int b) {
    if (r >= MIN_RED_CONE && r <= MAX_RED_CONE && g >= MIN_GREEN_CONE && g <= MAX_GREEN_CONE && b >= MIN_BLUE_CONE
        && b <= MAX_BLUE_CONE)
      return GamePiece.CONE;
    else if (r >= MIN_RED_CUBE && r <= MAX_RED_CUBE && g >= MIN_GREEN_CUBE && g <= MAX_GREEN_CUBE && b >= MIN_BLUE_CUBE
        && b <= MAX_BLUE_CUBE)
      return GamePiece.CUBE;
    else
      return GamePiece.NONE;
  }

  public enum GamePiece {
    NONE,
    CONE,
    CUBE
  }

  public void setOpen() {
    gripperSolenoid.set(Value.kForward);
  }

  public void setClosed() {
    gripperSolenoid.set(Value.kReverse);
  }
 
  // Pistons that push/pull gripper claws -- toggles position
  public void GripperToggle() {
    // if (!ph.getPressureSwitch() && ph.getCompressor())
    if (gripperSolenoid.get() == Value.kOff) {
      gripperSolenoid.set(Value.kForward);
    } else {
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

  public int lightingInfo(String lightingInput) {
    return lightController.writeString(lightingInput);
  }

  public GamePiece detectGamePiece()
  {
    TCS34725_RGB color = colorSensor.getRGB();
    return checkColors(color.getR(), color.getG(),color.getB());
  }
}
