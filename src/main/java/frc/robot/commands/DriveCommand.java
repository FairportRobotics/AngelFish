package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {

    private DriveSubsystem driveSubsystem;
    private CommandXboxController controller;

    /**
     * Construct a new drive command.
     */
    public DriveCommand(CommandXboxController controller, DriveSubsystem driveSubsystem) {
        this.controller = controller;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double forward = controller.getLeftY();
        double strafe = controller.getLeftX();
        double rotate = controller.getRightX();
        forward = deadband(forward, Constants.DEADBAND_TRANSLATE);
        strafe = deadband(strafe, Constants.DEADBAND_TRANSLATE);
        rotate = deadband(rotate, Constants.DEADBAND_ROTATE);
        forward *= Math.abs(forward);
        strafe *= Math.abs(strafe);
        rotate *= Math.abs(rotate);
        double speed = controller.getLeftTriggerAxis() > 0.5 ? Constants.FAST_SPEED : Constants.SLOW_SPEED;
        ChassisSpeeds chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(forward * speed, strafe * speed, rotate, driveSubsystem.getRotation());
        driveSubsystem.drive(chassisSpeed);
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