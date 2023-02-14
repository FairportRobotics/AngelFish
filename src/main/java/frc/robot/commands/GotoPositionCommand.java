package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class GotoPositionCommand extends CommandBase {

    private DriveSubsystem driveSubsystem;
    private DriveSubsystem gyro;
    private double x;
    private double y;
    ProfiledPIDController xController;
    ProfiledPIDController yController;

    /**
     * Construct a command to drive to robot to a target position.
     * @param driveSubsystem Drive subsystem.
     * @param gyro Gyroscope.
     * @param x Target X position.
     * @param y Target Y position.
     */
    public GotoPositionCommand(DriveSubsystem driveSubsystem, DriveSubsystem gyro, double x, double y) {
        this.gyro = gyro;
        this.driveSubsystem = driveSubsystem;
        this.x = x;
        this.y = y;
        
        xController = new ProfiledPIDController(Constants.GO_TO_POSITION_PID_P, Constants.GO_TO_POSITION_PID_I, Constants.GO_TO_POSITION_PID_D, 
            new TrapezoidProfile.Constraints(Constants.GO_TO_POSITION_MAX_VELOCITY, Constants.GO_TO_POSITION_MAX_ACCELERATION));

        yController = new ProfiledPIDController(Constants.GO_TO_POSITION_PID_P, Constants.GO_TO_POSITION_PID_I, Constants.GO_TO_POSITION_PID_D,
            new TrapezoidProfile.Constraints(Constants.GO_TO_POSITION_MAX_VELOCITY, Constants.GO_TO_POSITION_MAX_ACCELERATION));
        
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        Pose2d currLoc = driveSubsystem.getPosition();
        double outX = xController.getSetpoint().velocity + xController.calculate(currLoc.getX(), x);
        double outY = yController.getSetpoint().velocity + yController.calculate(currLoc.getY(), y);
        driveSubsystem.drive(outX, outY, 0, gyro.getYaw()); //Turned red after transferring GyroSubsytem into DriveSubsytem. Method seems to call itself in DriveSubsystem.
    }


    /**
     * Test if the robot is close enough to the target location, and travelling at a low enough speed.
     * @return is finished
     */
    @Override
    public boolean isFinished() {
        Pose2d currLoc = driveSubsystem.getPosition();
        return (driveSubsystem.getAverageVelocity() < Constants.GO_TO_VELOCITY_ERROR) && 
            (Math.sqrt(Math.pow(currLoc.getX() - x, 2) + Math.pow(currLoc.getY() - y, 2)) < Constants.GO_TO_POSITION_ERROR);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, 0);
    }

}