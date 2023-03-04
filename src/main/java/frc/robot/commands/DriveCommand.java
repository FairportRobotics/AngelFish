package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Util;
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

        forward = Util.deadband(forward, Constants.DEADBAND_TRANSLATE);
        strafe = Util.deadband(strafe, Constants.DEADBAND_TRANSLATE);
        rotate = Util.deadband(rotate, Constants.DEADBAND_ROTATE);

        forward *= Math.abs(forward);
        strafe *= Math.abs(strafe);
        rotate *= Math.abs(rotate);

        double speed = controller.getLeftTriggerAxis() > 0.5 ? Constants.FAST_SPEED : Constants.SLOW_SPEED;

        ChassisSpeeds chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(-forward * speed, strafe * speed, rotate * Constants.ROTATION_RATE, driveSubsystem.getRotation());
        
        driveSubsystem.drive(chassisSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}