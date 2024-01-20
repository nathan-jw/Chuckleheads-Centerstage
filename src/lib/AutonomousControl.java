package org.firstinspires.ftc.teamcode.lib;


/**
 * For telling the robot what side and alliance it is on.
 *
 * @author Nathan W
 */
public class AutonomousControl {
    /**
     * Represents the alliances that the robot can start on
     */
    public enum Alliance {
        /**
         * red alliance
         */
        RED,
        /**
         * blue alliance
         */
        BLUE,
    }


    /**
     * Represents the sides of the field that the robot can start on
     */
    public enum Side {
        /**
         * left side of the field, looking at it from your alliance's side
         */
        LEFT,
        /**
         * right side of the field, looking at it from your alliance's side
         */
        RIGHT,
    }
}