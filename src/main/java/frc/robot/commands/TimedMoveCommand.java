package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.DriveSubsystem;
//import frc.robot.subsystems.LightingSubsystem;

public class TimedMoveCommand extends CommandBase {

    public double targetX;
    public double targetY;
    public int targetRotate;

    private PIDController pidControllerRotation;

    private DriveSubsystem m_driveSubsystem;
    //private GyroSubsystem m_gryoSubsystem;
    //private LightingSubsystem lightingSubsystem;
    private boolean isRotateDone = false;

    int millis = 0;

    long startTime = 0;

    private double error;

    // public NewAutoMoveToCommand(double x, double y, int rotate, int millis) {
    //     this.targetX = x;
    //     this.targetY = y;
    //     this.targetRotate = rotate;
    //     this.millis = millis;

    //     this.m_driveSubsystem = RobotContainer.getInstance().driveSubsystem;
    //     this.m_gryoSubsystem = RobotContainer.getInstance().gyroSubsystem;
    //     this.lightingSubsystem = RobotContainer.getInstance().lightingSubsystem;

    //     pidControllerRotation = new PIDController(.025, .001, .005);
    // }

    public TimedMoveCommand(double x, double y, int millis, DriveSubsystem driveSubsystem) {
        this.targetX = x;
        this.targetY = y;
        this.targetRotate = 0;
        this.millis = millis;

        this.m_driveSubsystem = driveSubsystem;
        // this.m_gryoSubsystem = RobotContainer.getInstance().gyroSubsystem;
        // this.lightingSubsystem = RobotContainer.getInstance().lightingSubsystem;

        pidControllerRotation = new PIDController(.025, .0, .005);
        pidControllerRotation.setIntegratorRange(-0.1, 0.1);
    }

    @Override
    public void initialize() {
        // add second
        pidControllerRotation.setSetpoint(targetRotate);
        this.startTime = System.currentTimeMillis();
        targetRotate = (int)m_driveSubsystem.getYaw();
        m_driveSubsystem.drive(0, 0, 0, 0);
    }

    public void execute() {

        double pidOutRotation = 0;
        if (!isRotateDone)
            pidOutRotation = pidControllerRotation.calculate(m_driveSubsystem.getYaw());

        m_driveSubsystem.drive(-targetX, -targetY, -pidOutRotation, m_driveSubsystem.getYaw());

        if(Math.abs(pidControllerRotation.getPositionError()) <= 0.5)
            isRotateDone = true;

        // Pose2d pose = m_driveSubsystem.getPose();

        // //m_driveSubsystem.move(-pidOutX, -pidOutY, -pidOutRotation,
        // m_gryoSubsystem.getYaw());
        // //m_driveSubsystem.move(-pidControllerX.ge, -pidOutY, 0,
        // m_gryoSubsystem.getYaw());
        // double xMove = pidControllerX.calculate(pose.getX() + targetX);
        // double yMove = pidControllerY.calculate(pose.getY() + targetY);
        // SmartDashboard.putNumber("X pose", pose.getX());
        // SmartDashboard.putNumber("Y pose", pose.getY());
        // SmartDashboard.putNumber("X", xMove);
        // SmartDashboard.putNumber("Y", yMove);
        // m_driveSubsystem.move(yMove, xMove, 0, m_gryoSubsystem.getYaw());
        // error = pidControllerY.getPositionError();
    }

    @Override
    public boolean isFinished() {
        System.out.println("error: ");
        System.out.println(error);
        return System.currentTimeMillis() - startTime >= millis;
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.drive(0, 0, 0, m_driveSubsystem.getYaw());
    }
}