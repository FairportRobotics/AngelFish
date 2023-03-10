package frc.robot.subsystems;

import java.util.Comparator;
import java.util.Optional;
import java.util.stream.Stream;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.fairportrobotics.frc.poe.CameraTracking.RobotFieldPosition;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class DriveSubsystem extends SubsystemBase {
    private static final double MAX_VOLTAGE = 12.0;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.14528;
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private final AHRS gyro = new AHRS();

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0));
    private final SwerveDriveOdometry odometry;

    private RobotFieldPosition frontCameraRobotFieldPosition;
    private RobotFieldPosition backCameraRobotFieldPosition;

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private boolean locked;

    private final Field2d m_field = new Field2d();

    private SlewRateLimiter simVelocityX;
    private SlewRateLimiter simVelocityY;

    public DriveSubsystem() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");

        frontLeftModule = new MkSwerveModuleBuilder()
            .withLayout(shuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(0, 0))
            .withGearRatio(SdsModuleConfigurations.MK4I_L1)
            .withDriveMotor(MotorType.FALCON, Constants.FRONT_LEFT_DRIVE_ID)
            .withSteerMotor(MotorType.FALCON, Constants.FRONT_LEFT_SWERVE_ID)
            .withSteerEncoderPort(Constants.FRONT_LEFT_ENCODER_ID)
            .withSteerOffset(Constants.FRONT_LEFT_SWERVE_OFFSET)
            .build();

        frontRightModule = new MkSwerveModuleBuilder()
            .withLayout(shuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(2, 0))
            .withGearRatio(SdsModuleConfigurations.MK4I_L1)
            .withDriveMotor(MotorType.FALCON, Constants.FRONT_RIGHT_DRIVE_ID)
            .withSteerMotor(MotorType.FALCON, Constants.FRONT_RIGHT_SWERVE_ID)
            .withSteerEncoderPort(Constants.FRONT_RIGHT_ENCODER_ID)
            .withSteerOffset(Constants.FRONT_RIGHT_SWERVE_OFFSET)
            .build();

        backLeftModule = new MkSwerveModuleBuilder()
            .withLayout(shuffleboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(4, 0))
            .withGearRatio(SdsModuleConfigurations.MK4I_L1)
            .withDriveMotor(MotorType.FALCON, Constants.BACK_LEFT_DRIVE_ID)
            .withSteerMotor(MotorType.FALCON, Constants.BACK_LEFT_SWERVE_ID)
            .withSteerEncoderPort(Constants.BACK_LEFT_ENCODER_ID)
            .withSteerOffset(Constants.BACK_LEFT_SWERVE_OFFSET)
            .build();

        backRightModule = new MkSwerveModuleBuilder()
            .withLayout(shuffleboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(6, 0))
            .withGearRatio(SdsModuleConfigurations.MK4I_L1)
            .withDriveMotor(MotorType.FALCON, Constants.BACK_RIGHT_DRIVE_ID)
            .withSteerMotor(MotorType.FALCON, Constants.BACK_RIGHT_SWERVE_ID)
            .withSteerEncoderPort(Constants.BACK_RIGHT_ENCODER_ID)
            .withSteerOffset(Constants.BACK_RIGHT_SWERVE_OFFSET)
            .build();

        ((WPI_TalonFX) frontLeftModule.getDriveMotor()).setNeutralMode(NeutralMode.Brake);
        ((WPI_TalonFX) frontRightModule.getDriveMotor()).setNeutralMode(NeutralMode.Brake);
        ((WPI_TalonFX) backLeftModule.getDriveMotor()).setNeutralMode(NeutralMode.Brake);
        ((WPI_TalonFX) backRightModule.getDriveMotor()).setNeutralMode(NeutralMode.Brake);


        odometry = new SwerveDriveOdometry(
            kinematics,
            Rotation2d.fromDegrees(gyro.getYaw()),
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            }
        );

        try { 
            frontCameraRobotFieldPosition = new RobotFieldPosition("front-facing", Constants.FRONT_CAM_TO_CENTER, AprilTagFields.k2023ChargedUp.m_resourceFile, PoseStrategy.CLOSEST_TO_LAST_POSE);
            //backCameraRobotFieldPosition = new RobotFieldPosition("back-facing", Constants.FRONT_CAM_TO_CENTER, AprilTagFields.k2023ChargedUp.m_resourceFile, PoseStrategy.CLOSEST_TO_LAST_POSE);
        } catch (Exception e) {
            System.out.println("Failed To Load AprilTagLayout (basically we don't have april tag based file positioning)");
            e.printStackTrace();
        }

        this.locked = false;

        shuffleboardTab.addNumber("Gyroscope Angle", () -> getRotation().getDegrees());
        shuffleboardTab.addNumber("Pose X", () -> odometry.getPoseMeters().getX());
        shuffleboardTab.addNumber("Pose Y", () -> odometry.getPoseMeters().getY());
        SmartDashboard.putData("Field", m_field);
        simVelocityX = new SlewRateLimiter(10);
        simVelocityY = new SlewRateLimiter(10);
    }

    public void zeroGyroscope() {
        odometry.resetPosition(
            Rotation2d.fromDegrees(gyro.getYaw()),
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition() },
            new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0)
            )
        );
    }

    public void resetOdometry(Pose2d newPosition) {
        odometry.resetPosition(
            newPosition.getRotation(),
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            },
            new Pose2d(newPosition.getTranslation(), newPosition.getRotation())
        );
    }

    public Rotation2d getRotation() {
        return odometry.getPoseMeters().getRotation();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }

    public void lock() {
        this.locked = true;
    }

    public void unlock() {
        this.locked = false;
    }

    @Override
    public void periodic() {
        odometry.update(
            Robot.isReal() ? Rotation2d.fromDegrees(gyro.getYaw()) : getRotation(),
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            }
        );

        EstimatedRobotPose resultToUse = null;
        double lowestAmbiguity = 1;
        //We should always have a front camera (front > back)
        if(frontCameraRobotFieldPosition != null) {
            //Set the pose and freeze the positions to prevent issues
            frontCameraRobotFieldPosition.setLastPose(odometry.getPoseMeters());
            frontCameraRobotFieldPosition.freeze();

            if(backCameraRobotFieldPosition != null) {
                backCameraRobotFieldPosition.setLastPose(odometry.getPoseMeters());
                backCameraRobotFieldPosition.freeze();
            }

            // If both cameras have a result, use the one with the highest ambiguity target inside
            if(frontCameraRobotFieldPosition.getEstimatedGlobalPose().isPresent() && 
                    backCameraRobotFieldPosition != null && 
                    backCameraRobotFieldPosition.getEstimatedGlobalPose().isPresent()) {



                EstimatedRobotPose frontCameraPose = frontCameraRobotFieldPosition.getEstimatedGlobalPose().get();
                EstimatedRobotPose backCameraPose = backCameraRobotFieldPosition.getEstimatedGlobalPose().get();

                //Get the pose with the lowest ambiguity target
                for(PhotonTrackedTarget target : frontCameraPose.targetsUsed) {
                    if(target.getPoseAmbiguity() < lowestAmbiguity) {
                        resultToUse = frontCameraPose;
                        lowestAmbiguity = target.getPoseAmbiguity();
                    }
                }

                for(PhotonTrackedTarget target : backCameraPose.targetsUsed) {
                    if(target.getPoseAmbiguity() < lowestAmbiguity) {
                        resultToUse = backCameraPose;
                        lowestAmbiguity = target.getPoseAmbiguity();
                    }
                }

                //If only the front camera returned a position
            } else if(frontCameraRobotFieldPosition.getEstimatedGlobalPose().isPresent()) {
                for(PhotonTrackedTarget target : frontCameraRobotFieldPosition.getEstimatedGlobalPose().get().targetsUsed) {
                    if(target.getPoseAmbiguity() < lowestAmbiguity) {
                        lowestAmbiguity = target.getPoseAmbiguity();
                    }
                }
                resultToUse = frontCameraRobotFieldPosition.getEstimatedGlobalPose().get();

                //If only the back camera returned a position
            } else if(backCameraRobotFieldPosition != null && backCameraRobotFieldPosition.getEstimatedGlobalPose().isPresent()) {
                for(PhotonTrackedTarget target : backCameraRobotFieldPosition.getEstimatedGlobalPose().get().targetsUsed) {
                    if(target.getPoseAmbiguity() < lowestAmbiguity) {
                        lowestAmbiguity = target.getPoseAmbiguity();
                    }
                }
                resultToUse = backCameraRobotFieldPosition.getEstimatedGlobalPose().get();

            } 

            frontCameraRobotFieldPosition.unfreeze();
            if(backCameraRobotFieldPosition != null) {
                backCameraRobotFieldPosition.unfreeze();
            }
        }

        //TODO Factor in the position if ambiguity is above 0.05
        if(resultToUse != null && lowestAmbiguity < 0.05) {
            odometry.resetPosition(Rotation2d.fromDegrees(-gyro.getYaw()), new SwerveModulePosition[]{             
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()}, 
                new Pose2d(resultToUse.estimatedPose.getX(), resultToUse.estimatedPose.getY(), resultToUse.estimatedPose.getRotation().toRotation2d()));
        }



        if (locked) {
            frontLeftModule.set(0, Math.PI/4);
            frontRightModule.set(0, 3*Math.PI/4);
            backLeftModule.set(0, Math.PI/4);
            backRightModule.set(0, 3*Math.PI/4);
        } else {
            frontLeftModule.set(0, 0);
            SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
            frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
            frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
            backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
            backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
        }
        m_field.setRobotPose(getPose());
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                // Reset odometry for the first path you run during auto
                if (isFirstPath) {
                    this.resetOdometry(traj.getInitialHolonomicPose());
                }
            }),
            new PPSwerveControllerCommand(
                traj,
                this::getPose,
                new PIDController(0.1, 0, 0), // X controller
                new PIDController(0.1, 0, 0), // Y controller
                new PIDController(0.1, 0, 0), // Rotation controller
                this::drive,
                true,
                this
            )
        );
    }

    @Override
    public void simulationPeriodic() {
        if (locked) { return; }
        double x = getPose().getX();
        double y = getPose().getY();
        Rotation2d rotation = getRotation();

        double targetVelX = (chassisSpeeds.vxMetersPerSecond*Math.cos(rotation.getRadians())-chassisSpeeds.vyMetersPerSecond*Math.sin(rotation.getRadians()));
        double targetVelY = (chassisSpeeds.vxMetersPerSecond*Math.sin(rotation.getRadians())+chassisSpeeds.vyMetersPerSecond*Math.cos(rotation.getRadians()));
        Rotation2d dRotation = Rotation2d.fromRadians(-chassisSpeeds.omegaRadiansPerSecond*0.02);

        x += simVelocityX.calculate(targetVelX)*0.02;
        y += simVelocityY.calculate(targetVelY)*0.02;
        rotation = rotation.rotateBy(dRotation);

        this.resetOdometry(new Pose2d(x, y, rotation));
    }
}