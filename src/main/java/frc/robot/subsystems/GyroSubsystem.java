package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

public class GyroSubsystem {
    private AHRS gyro;

    /**
     * Construct a new NAVX.
     */
    public GyroSubsystem() {
        gyro = new AHRS();
    }

    /**
     * Get the current yaw of the gyroscope.
     * @return current yaw in degrees.
     */
    public double getYaw() {
        return gyro.getYaw();
    }

    /**
     * Set the gyro's current direction to zero.
     */
    public void reset() {
        gyro.reset();
    }
}