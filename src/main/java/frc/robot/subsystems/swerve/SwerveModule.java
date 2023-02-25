package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {

    private WPI_TalonFX driveFalcon;
    private WPI_TalonFX swerveFalcon;
    private CANCoder encoder;

    private double offset;

    private PIDController driveController;
    private ProfiledPIDController swerveController;

    private SimpleMotorFeedforward driveFeedForward;
    private SimpleMotorFeedforward swerveFeedForward;

    private String name;

    /**
     * Create a new swerve module class.
     * @param driveFalconID ID of the drive falcon to set the speed of.
     * @param swerveFalconID ID of the swerve falcon to set the direction of.
     * @param encoderID ID of the encoder that gets the swerve angle.
     * @param name Name of the swerve module, used if smartdashboard.
     * @param offset Difference between true north and the hardware encoder angle, in degrees.
     */
    public SwerveModule(int driveFalconID, int swerveFalconID, int encoderID, String name, double offset) {
        driveFalcon = new WPI_TalonFX(driveFalconID);
        swerveFalcon = new WPI_TalonFX(swerveFalconID);
        encoder = new CANCoder(encoderID);

        this.offset = offset;
        this.name = name;

        driveController = new PIDController(Constants.SWERVE_DRIVE_P, Constants.SWERVE_DRIVE_I, Constants.SWERVE_DRIVE_D);
        swerveController = new ProfiledPIDController(Constants.SWERVE_STEER_P, Constants.SWERVE_STEER_I, Constants.SWERVE_STEER_D,
            new TrapezoidProfile.Constraints(Constants.MAX_ANG_VEL, Constants.MAX_ANG_ACC)
        );
        swerveController.enableContinuousInput(0, 360);
        encoder.setPositionToAbsolute();
        swerveFalcon.setInverted(true);
        driveFeedForward = new SimpleMotorFeedforward(0, 0);
        swerveFeedForward = new SimpleMotorFeedforward(0, 0);
    }
    /**
     * Get the angle of the module, taking the offset into account.
     * @return the direction the swerve module is pointing.
     */
    public double getAngle() {
        return encoder.getPosition() - offset;
    }

    /**
     * Get the position of swerve module, consisting of a distance and a direction.
     * @return <code>SwerveModulePosition</code> of the current module.
     */
    public SwerveModulePosition getPosition() {
        double distance = driveFalcon.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_METER;
        SmartDashboard.putNumber(name + "Distance Meters", distance);
        return new SwerveModulePosition(distance, Rotation2d.fromDegrees(getAngle()));
    }

    /**
     * Get the current speed of the swerve module, in m/s.
     * @return encoder velocity converted to m/s.
     */
    public double getVelocity() {
        return driveFalcon.getSelectedSensorVelocity() * 10 / Constants.ENCODER_TICKS_PER_METER;
    }

    /**
     * Set the swerve module to a <code>SwerveModuleState</code>.
     * @param state target state for the swerve module to be in.
     */
    public void fromModuleState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getAngle()));
        double driveFalconVoltage = driveController.calculate(getVelocity(),state.speedMetersPerSecond);
        double swerveFalconVoltage = swerveController.calculate(getAngle(), state.angle.getDegrees()) + swerveFeedForward.calculate(swerveController.getSetpoint().velocity);
        
        driveFalcon.set(ControlMode.PercentOutput,driveFalconVoltage);
        swerveFalcon.setVoltage(swerveFalconVoltage);

        
        //SmartDashboard.putNumber(name + " Swerve Voltage", swerveFalconVoltage);
       // SmartDashboard.putNumber(name + " Drive Voltage", driveFalconVoltage);
       // SmartDashboard.putNumber(name + " Current Angle", getAngle() );
       // SmartDashboard.putNumber(name + " Current Velocity", getVelocity() );
       // SmartDashboard.putNumber(name + " Target Angle", state.angle.getDegrees());
      //  SmartDashboard.putNumber(name + " Target Velocity", state.speedMetersPerSecond);
       // SmartDashboard.putNumber(name + " Swerve Target Velocity", (swerveController.getSetpoint().velocity));
    }

}