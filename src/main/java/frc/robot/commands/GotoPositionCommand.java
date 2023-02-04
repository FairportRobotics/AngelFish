package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class GotoPositionCommand extends CommandBase {

    private DriveSubsystem driveSubsystem;
    private GyroSubsystem gyro;
    private double x;
    private double y;
    ProfiledPIDController xProfile;
    ProfiledPIDController yProfile;

    /**
     * Construct a command to drive to robot to a target position.
     * @param driveSubsystem Drive subsystem.
     * @param gyro Gyroscope.
     * @param x Target X position.
     * @param y Target Y position.
     */
    public GotoPositionCommand(DriveSubsystem driveSubsystem, GyroSubsystem gyro, double x, double y) {
        this.gyro = gyro;
        this.driveSubsystem = driveSubsystem;
        this.x = x;
        this.y = y;
        xProfile = new ProfiledPIDController(1, 0, 0.5, new TrapezoidProfile.Constraints(1, 0.5));
        yProfile = new ProfiledPIDController(1, 0, 0.5, new TrapezoidProfile.Constraints(1, 0.5));
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        Pose2d currLoc = driveSubsystem.getPosition();
        double outX = xProfile.getSetpoint().velocity + xProfile.calculate(currLoc.getX(), x);
        double outY = yProfile.getSetpoint().velocity + yProfile.calculate(currLoc.getY(), y);
        driveSubsystem.drive(outX, outY, 
        0, gyro.getYaw());
    }


    /**
     * Test if the robot is close enough to the target location, and travelling at a low enough speed.
     * @return is finished
     */
    @Override
    public boolean isFinished() {
        Pose2d currLoc = driveSubsystem.getPosition();
        return driveSubsystem.getAverageVelocity() < Constants.GO_TO_VELOCITY_ERROR && 
        Math.sqrt(Math.pow(currLoc.getX() - x, 2) + 
        Math.pow(currLoc.getY() - y, 2)) < Constants.GO_TO_POSITION_ERROR;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, 0);
    }

}