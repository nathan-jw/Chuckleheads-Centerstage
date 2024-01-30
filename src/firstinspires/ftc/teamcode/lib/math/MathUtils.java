package org.firstinspires.ftc.teamcode.lib.math;

import java.util.Arrays;

/**
 * Various helpful math-related functions.
 * @author Various
 */
public class MathUtils {
    /**
     * Squishes or expands the range a value is in from the old to new range, changing the value. If a value is 1 in the range [0, 2], it would be 2 in the range [0, 4] for example.
     * @param old_min Old range minimum
     * @param old_max Old range maximum
     * @param new_min New range minimum
     * @param new_max New range maximum
     * @param value Value to remap
     * @return The remapped value
     */
    public static double remapRange(double old_min, double old_max, double new_min, double new_max, double value) {
        return new_min + (value - old_min) * ((new_max - new_min) / (old_max - old_min));
    }


    /**
     * clamps a value in a range. Returns min if value < min, max if value > max, and value otherwise
     * @param min minimum value in range
     * @param max maximum value in range
     * @param value value that needs to be clamped
     * @return the clamped value
     */
    public static double clamp(double min, double max, double value) {
        if (value > max) { return max; }
        if (value < min) { return min; }
        return value;
    }


    /**
     * squares an int
     * @param x the number to square
     * @return the int, squared
     */
    public static int sqr(int x) {
        return x * x;
    }

    /**
     * squares a double
     * @param x the number to square
     * @return the double, squared
     */
    public static double sqr(double x) {
        return x * x;
    }


    /**
     * returns median value in an array
     * @param a integer array
     * @return the median value of the array
     */
    public static double findMedian(int a[]) {
        int n = a.length;

        Arrays.sort(a);

        // check for even case
        if (n % 2 != 0) {
            return (double) a[n / 2];
        }

        return (double)(a[(n - 1) / 2] + a[n / 2]) / 2.0;
    }


    /**
     * returns the number in an array closest to the given value
     * @param nums array of integers to compare to value
     * @param value number to compare to
     * @return returns double closest to value
     */
    public static int smallestDiff(int nums[], int value) {
        int smallestDifference = 250;
        int smallestValue = nums[0];
        for (int i = 0; i < nums.length; i++) {
            int difference = Math.abs(nums[i] - value);
            if (difference < smallestDifference) {
                smallestDifference = difference;
                smallestValue = nums[i];
            }
        }
        return smallestValue;
    }


    /**
     * Finds a value from -1 to 1 representing how much the robot needs to turn to reach the given target angle
     * @param currentAngle the current angle of the robot in degrees
     * @param targetAngle than angle in degrees you want the robot to face
     * @return Value from -1 to 1 representing how much the robot should turn to reach the current angle.
     * This value can be plugged as the turn constant in the driving code
     */
    public static double shortestAngleRemapped(double targetAngle, double currentAngle) {
        double moveConst = 0;

        if (currentAngle > 0) {
            moveConst = -1 * (Math.abs(currentAngle) - targetAngle);
        }
        else {
            moveConst = -1 * (currentAngle - targetAngle);
        }

        double other = (-1 * moveConst/Math.abs(moveConst)) * ((Math.PI*2) - Math.abs(moveConst));
        if (Math.abs(other) < Math.abs(moveConst)) {
            return other;
        }
        else {
            return moveConst;
        }
    }
}
