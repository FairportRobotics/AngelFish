package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class DriveCommand extends CommandBase {

    private DriveSubsystem driveSubsystem;
	private GyroSubsystem gyroSubsystem;
    private CommandXboxController controller;

    /**
     * Construct a new drive command.
     */
    public DriveCommand() {
        controller = RobotContainer.getInstance().controller;
		driveSubsystem = RobotContainer.getInstance().driveSubsystem;
		gyroSubsystem = RobotContainer.getInstance().gyroSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double forward = controller.getLeftY();
        double strafe = controller.getLeftX();
        double rotate = controller.getRightX();
        forward = deadband(forward, 0.05);
        strafe = deadband(strafe, 0.05);
        rotate = deadband(rotate, 0.05);
        forward *= Math.abs(forward);
        strafe *= Math.abs(strafe);
        rotate *= Math.abs(rotate);
        driveSubsystem.drive(forward * 5, strafe * 5, rotate, gyroSubsystem.getYaw());
    }

    /**
     * Remove small inputs from the controllers.
     * @param value current controller value.
     * @param deadband cutoff for deadband.
     * @return deadbanded value.
     */
    private static double deadband(double value, double deadband) {
        if(Math.abs(value) > deadband) {
            if(value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}