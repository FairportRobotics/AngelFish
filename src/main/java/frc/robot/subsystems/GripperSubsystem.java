package frc.robot.subsystems;

import com.fairportrobotics.frc.poe.sensors.colorsensors.TCS34725;
import com.fairportrobotics.frc.poe.sensors.colorsensors.TCS34725.TCS34725_RGB;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GripperSubsystem extends SubsystemBase {

    private DoubleSolenoid gripperSolenoid;
    private DoubleSolenoid brakeSolenoid;
    private PneumaticHub ph;
    private Compressor phCompressor;
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

    private Mechanism2d mechanism;
    private MechanismRoot2d rootLeft;
    private MechanismRoot2d rootRight;
    private MechanismLigament2d left;
    private MechanismLigament2d right;

    public GripperSubsystem(PneumaticHub ph) {

        gripperSolenoid = ph.makeDoubleSolenoid(Constants.PH_GRIPPER_OPEN, Constants.PH_GRIPPER_CLOSE);
        gripperSolenoid.set(Value.kForward);

        colorSensor = new TCS34725();

        mechanism = new Mechanism2d(3, 2);

        rootLeft = mechanism.getRoot("left", 1, 0);
        rootRight = mechanism.getRoot("right", 2, 0);

        left = rootLeft.append(new MechanismLigament2d("left", 1.5, 70));
        right = rootRight.append(new MechanismLigament2d("right", 1.5, 110));

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
    SmartDashboard.putData("Gripper", mechanism);

  }
    public GamePiece checkColors(int r, int g, int b) {
        if (r >= Constants.MIN_RED_CONE && r <= Constants.MAX_RED_CONE
                && g >= Constants.MIN_GREEN_CONE && g <= Constants.MAX_GREEN_CONE
                && b >= Constants.MIN_BLUE_CONE && b <= Constants.MAX_BLUE_CONE) {
            return GamePiece.CONE;
        } else if (r >= Constants.MIN_RED_CUBE && r <= Constants.MAX_RED_CUBE
                && g >= Constants.MIN_GREEN_CUBE && g <= Constants.MAX_GREEN_CUBE
                && b >= Constants.MIN_BLUE_CUBE && b <= Constants.MAX_BLUE_CUBE) {
            return GamePiece.CUBE;
        } else {
            return GamePiece.NONE;
        }
    }

    public enum GamePiece {
        NONE,
        CONE,
        CUBE
    }

    public void setOpen() {
        gripperSolenoid.set(Value.kForward);
        left.setAngle(110);
        right.setAngle(70);
    }

    public void setClosed() {
        gripperSolenoid.set(Value.kReverse);
        left.setAngle(70);
        right.setAngle(110);
    }

    public int lightingInfo(String lightingInput) {
        return lightController.writeString(lightingInput);
    }

    public GamePiece detectGamePiece() {
        TCS34725_RGB color = colorSensor.getRGB();
        return checkColors(color.getR(), color.getG(), color.getB());
    }
}