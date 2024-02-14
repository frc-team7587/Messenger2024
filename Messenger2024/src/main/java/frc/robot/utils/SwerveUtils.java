package frc.robot.utils;

public class SwerveUtils {
    /**
     * Steps a value towards a target with a specified step size.
     * @param current The current or starting value.  Can be positive or negative.
     * @param target The target value the algorithm will step towards.  Can be positive or negative.
     * @param stepSize The maximum step size that can be taken.
     * @return The new value for {@code current} after performing the specified step towards the specified target.
     */
    public static double stepTowards(double current, double target, double stepSize) {
        if (Math.abs(current - target) <= stepSize) {
            return target;
        } else if (target < current) {
            return current - stepSize;
        } else {
            return current + stepSize;
        }
    }

    /**
     * Steps a value (angle) towards a target (angle) taking the shortest path with a specified step size.
     * @param current The current or starting angle (in radians).  Can lie outside the 0 to 2 * PI range.
     * @param target The target angle (in radians) the algorithm will step towards.  Can lie outside the 0 to 2 * PI range.
     * @param stepSize The maximum step size that can be taken (in radians).
     * @return The new angle (in radians) for {@code current} after performing the specified step towards the specified target.
     * This value will always lie in the range 0 to 2*PI (exclusive).
     */
    public static double stepTowardsCircular(double current, double target, double stepSize) {
        current = wrapAngle(current);
        target = wrapAngle(target);

        double stepDirection = Math.signum(target - current);
        double difference = Math.abs(current - target);
        
        if (difference <= stepSize) {
            return target;
        } else if (difference > Math.PI) { // Does the system need to wrap over eventually?
            // Handle the special case where you can reach the target in one step while also wrapping
            if (current + 2 * Math.PI - target < stepSize || target + 2 * Math.PI - current < stepSize) {
                return target;
            } else {
                return wrapAngle(current - stepDirection * stepSize); //this will handle wrapping gracefully
            }
        } else {
            return current + stepDirection * stepSize;
        }
    }

    /**
     * Finds the (unsigned) minimum difference between two angles including calculating across 0.
     * @param angleA An angle (in radians).
     * @param angleB An angle (in radians).
     * @return The (unsigned) minimum difference between the two angles (in radians).
     */
    public static double angleDifference(double angleA, double angleB) {
        double difference = Math.abs(angleA - angleB);
        return difference > Math.PI? (2 * Math.PI) - difference : difference;
    }

    /**
     * Wraps an angle until it lies within the range from 0 to 2 * PI (exclusive).
     * @param angle The angle (in radians) to wrap. Can be positive or negative and can lie multiple wraps outside the output range.
     * @return An angle (in radians) from 0 and 2 * PI (exclusive).
     */
    public static double wrapAngle(double angle) {
        double twoPi = 2*Math.PI;

        if (angle == twoPi) { // Handle this case separately to avoid floating point errors with the floor after the division in the case below
            return 0.0;
        } else if (angle > twoPi) {
            return angle - twoPi * Math.floor(angle / twoPi);
        } else if (angle < 0.0) {
            return angle + twoPi * (Math.floor((-angle) / twoPi) + 1);
        } else {
            return angle;
        }
    }
}