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
}
