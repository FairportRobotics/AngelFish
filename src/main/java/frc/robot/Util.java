package frc.robot;

public class Util {
    /**
     * Linearly map a value from one range to another.
     * @param val input value
     * @param inputMin minimum of the input range
     * @param inputMax maximum of the input range
     * @param outputMin minimum of the output range
     * @param outputMax maximum of the output range
     * @return the mapped value
     */
    public static double map(double val, double inputMin, double inputMax, double outputMin, double outputMax) {
        return (outputMax*inputMin - outputMax*val - outputMin*inputMax + outputMin*val)/(inputMin - inputMax);
    }

    /**
     * Remove small inputs from the controllers.
     * @param value current controller value.
     * @param deadband cutoff for deadband.
     * @return deadbanded value.
     */
    public static double deadband(double value, double deadband) {
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
}
