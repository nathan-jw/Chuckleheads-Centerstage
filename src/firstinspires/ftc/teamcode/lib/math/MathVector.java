package org.firstinspires.ftc.teamcode.lib.math;

/**
 * Represents a vector
 * @author Neil N
 */
public class MathVector {
    /**
     * vector magnitude
     */
    private double magnitude;
    /**
     * vector angle
     */
    private double angle;
    /**
     * vector x position
     */
    private double x;
    /**
     * vector y position
     */
    private double y;


    /**
     * creates vector
     * @param m magnitude
     * @param a angle
     */
    public MathVector(double m, double a) {
        if (m < 0) {
            this.magnitude = -m;
            this.angle = a + Math.PI;
        }
        else {
            this.magnitude = m;
            this.angle = a;
        }

        this.x = Math.cos(a) * m;
        this.y = Math.sin(a) * m;
    }


    /**
     * adds an array of vectors together
     * @param vectors vector array to be added
     * @return returns sum of all vectors
     */
    public static MathVector add(MathVector[] vectors) {
        double totaly = 0;
        double totalx = 0;
        for (int i = 0; i < vectors.length; i++) {
            totalx += vectors[i].x;
            totaly += vectors[i].y;
        }

        double angle = Math.atan2(totaly, totalx);
        double magnitude = Math.sqrt((totalx*totalx)+(totaly*totaly));
        return new MathVector(magnitude, angle);
    }


    /*** GETTERS ***/
    /**
     * Get the vector magnitude.
     * @return The vector magnitude.
     */
    public double getMagnitude() { return magnitude; }

    /**
     * Get the vector angle.
     * @return The vector angle.
     */
    public double getAngle() { return angle; }

    /**
     * Get the vector X position.
     * @return The vector X position.
     */
    public double getX() { return x; }

    /**
     * Get the vector Y position.
     * @return The vector Y position.
     */
    public double getY() { return y; }
}