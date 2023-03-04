package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GripperSubsystem extends SubsystemBase {

    private DoubleSolenoid gripperSolenoid;
    private PneumaticHub ph;
    private Compressor phCompressor;

    private Mechanism2d mechanism;
    private MechanismRoot2d rootLeft;
    private MechanismRoot2d rootRight;
    private MechanismLigament2d left;
    private MechanismLigament2d right;

    public GripperSubsystem() {
        ph = new PneumaticHub(Constants.PH_CAN_ID);
        phCompressor = ph.makeCompressor();
        phCompressor.enableDigital();

        gripperSolenoid = ph.makeDoubleSolenoid(Constants.PH_GRIPPER_OPEN, Constants.PH_GRIPPER_CLOSE);
        gripperSolenoid.set(Value.kForward);

        mechanism = new Mechanism2d(3, 2);

        rootLeft = mechanism.getRoot("left", 1, 0);
        rootRight = mechanism.getRoot("right", 2, 0);

        left = rootLeft.append(new MechanismLigament2d("left", 1.5, 70));
        right = rootRight.append(new MechanismLigament2d("right", 1.5, 110));
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

    @Override
    public void periodic() {
        SmartDashboard.putData("Gripper", mechanism);
    }
}
