package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.GyroSubsystem;

public class DriveSubsystem extends SubsystemBase {

    private SwerveModule leftFrontModule;
    private SwerveModule rightFrontModule;
    private SwerveModule leftBackModule;
    private SwerveModule rightBackModule;

    private SwerveDriveKinematics kinematics;
    private GyroSubsystem gyro;

    private SwerveDriveOdometry odometry;

    /**
     * Construct a drive subsystem to drive the robot.
     * @param gyro Gyro to use to do odometry.
     */
    public DriveSubsystem(GyroSubsystem gyro) {
        this.gyro = gyro;

        leftFrontModule = new SwerveModule(Constants.FRONT_LEFT_DRIVE_ID, Constants.FRONT_LEFT_SWERVE_ID, Constants.FRONT_LEFT_ENCODER_ID, "Front Left", Constants.FRONT_LEFT_SWERVE_OFFSET);
        leftBackModule = new SwerveModule(Constants.BACK_LEFT_DRIVE_ID, Constants.BACK_LEFT_SWERVE_ID, Constants.BACK_LEFT_ENCODER_ID, "Front Right", Constants.FRONT_RIGHT_SWERVE_OFFSET);
        rightFrontModule = new SwerveModule(Constants.FRONT_RIGHT_DRIVE_ID, Constants.FRONT_RIGHT_SWERVE_ID, Constants.FRONT_RIGHT_ENCODER_ID, "Back Left", Constants.BACK_LEFT_SWERVE_OFFSET);
        rightBackModule = new SwerveModule(Constants.BACK_RIGHT_DRIVE_ID, Constants.BACK_RIGHT_SWERVE_ID, Constants.BACK_RIGHT_ENCODER_ID, "Back Right", Constants.BACK_RIGHT_SWERVE_OFFSET);

        Translation2d frontLeftLocation = new Translation2d(Constants.WHEEL_BASE/2, Constants.TRACK_WIDTH/2);
        Translation2d frontRightLocation = new Translation2d(Constants.WHEEL_BASE/2, Constants.TRACK_WIDTH/2);
        Translation2d backLeftLocation = new Translation2d(Constants.WHEEL_BASE/2, Constants.TRACK_WIDTH/2);
        Translation2d backRightLocation = new Translation2d(Constants.WHEEL_BASE/2, Constants.TRACK_WIDTH/2);

        kinematics = new SwerveDriveKinematics(
            frontLeftLocation,
            frontRightLocation,
            backLeftLocation,
            backRightLocation
        );


        odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(-gyro.getYaw()), new SwerveModulePosition[]{
            leftFrontModule.getPosition(),
            rightFrontModule.getPosition(),
            leftBackModule.getPosition(),
            rightBackModule.getPosition()
        });

       
    }

    /**
     * Drive the robot.
     * @param forward Target forward component of the robot in m/s
     * @param strafe Target sideways componoent of the robot in m/s
     * @param rotate Target rotation rate of the robot in rad/s
     * @param yaw Current gyro angle of the robot in degrees. Pass in 0 for robot oriented drive.
     */
    public void drive(double forward, double strafe, double rotate, double yaw) {
        SmartDashboard.putNumber("yaw", yaw);
        ChassisSpeeds velocity = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotate/10, Rotation2d.fromDegrees(-yaw));
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(velocity);
        leftFrontModule.fromModuleState(moduleStates[0]);
        leftBackModule.fromModuleState(moduleStates[1]);
        rightFrontModule.fromModuleState(moduleStates[2]);
        rightBackModule.fromModuleState(moduleStates[3]);
    }

     @Override
     public void periodic() {
        odometry.update(Rotation2d.fromDegrees(-gyro.getYaw()), new SwerveModulePosition[]{
            leftFrontModule.getPosition(),
            rightFrontModule.getPosition(),
            leftBackModule.getPosition(),
            rightBackModule.getPosition()
        });
        SmartDashboard.putNumber("Odometry X Meters", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry Y Meters", odometry.getPoseMeters().getY());
     }

     /**
      * Get the current pose from the odometry.
      * @return current pose.
      */
     public Pose2d getPosition() {
         return odometry.getPoseMeters();
     }

     /**
      * Approximation of the robot's current speed.
      * It approaches 0 when the robot's speed approaches 0
      * @return Approximation of the speed.
      */
     public double getAverageVelocity() {
         return (Math.abs( leftFrontModule.getVelocity()) +
                 Math.abs(rightFrontModule.getVelocity()) + 
                 Math.abs(  leftBackModule.getVelocity()) +
                 Math.abs( rightBackModule.getVelocity()))/4;
     }
}