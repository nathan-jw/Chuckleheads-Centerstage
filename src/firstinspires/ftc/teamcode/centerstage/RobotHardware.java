package org.firstinspires.ftc.teamcode.centerstage;

import org.firstinspires.ftc.teamcode.lib.MecanumChassis;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;


/**
 * Initializes hardware variables. Makes it so that one function call is
 * required in our teleop and autonomous files to get hardware variables.
 * @author Nathan W
 */
public class RobotHardware {
    /**
     * Mecanum chassis representing our drive motors, dead wheels, and imu
     */
    private MecanumChassis mc = null;

    /**
     * intake arm motor
     */
    // private DcMotorEx ia = null;
    // /**
    //  * intake slide motor
    //  */
    // private DcMotorEx is = null;
    // /**
    //  * FILL ME OUT
    //  */
    // private DcMotorEx slide = null;
    // /**
    //  * FILL ME OUT
    //  */
    // private DcMotorEx arm = null;

    // /**
    //  * left intake wrist servo
    //  */
    // private Servo iwl = null;
    // /**
    //  * right intake wrist servo
    //  */
    // private Servo iwr = null;
    // /**
    //  * left gripper
    //  */
    // private Servo gl = null;
    // /**
    //  * right gripper
    //  */
    // private Servo gr = null;


    /**
     * Initialize the hardware variables
     * @param hardwareMap HardwareMap from the Opmode
     */
    public RobotHardware(HardwareMap hardwareMap) {
        // Declare mecanum chassis
        mc = new MecanumChassis(
                hardwareMap.get(DcMotorEx.class, "frontLeft"),    // front left drive motor
                hardwareMap.get(DcMotorEx.class, "backLeft"),     // front right drive motor
                hardwareMap.get(DcMotorEx.class, "frontRight"),   // back left drive motor
                hardwareMap.get(DcMotorEx.class, "backRight"),    // back right drive motor
                hardwareMap.get(DcMotorEx.class, "frontLeft"),    // horizontal odometery pod
                hardwareMap.get(DcMotorEx.class, "frontRight"),   // vertical odometrty pod
                hardwareMap.get(BNO055IMU.class, "imu"));         // IMU

        // the rest of our motors
        // slide = hardwareMap.get(DcMotorEx.class, "slide");        // slide
        // ia = hardwareMap.get(DcMotorEx.class, "intakeArm");       // intake arm
        // is = hardwareMap.get(DcMotorEx.class, "intakeSpinner");   // instake spinner
        // arm = hardwareMap.get(DcMotorEx.class, "arm");            // worm gear arm
        // servos
        // iwl = hardwareMap.get(Servo.class, "intakeWristLeft");    // intake wrist left
        // iwr = hardwareMap.get(Servo.class, "intakeWristRight");   // intake wrist right
        // gl = hardwareMap.get(Servo.class, "lGrip");               // left gripper
        // gr = hardwareMap.get(Servo.class, "rGrip");               // right gripper

        // reset encoders
        // ia.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // is.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // reverse stuff
        // iwl.setDirection(Servo.Direction.REVERSE);
        // gr.setDirection(Servo.Direction.REVERSE);

        // set zero power behavior
        // arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        // ia.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        // slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }


    /**
     * Get the mecanum chassis.
     * @return The mecanum chassis.
     */
    public MecanumChassis getMc() { return mc; }
    /**
     * Get the slide motor.
     * @return The slide motor.
     */
    // public DcMotorEx getSlide() { return slide; }
    // /**
    //  * Get the intake arm motor.
    //  * @return The intake arm motor.
    //  */
    // public DcMotorEx getIa() { return ia; }
    // /**
    //  * Get the intake slide slide.
    //  * @return The intake arm motor.
    //  */
    // public DcMotorEx getIs() { return is; }
    // /**
    //  * Get the arm motor.
    //  * @return The arm motor.
    //  */
    // public DcMotorEx getArm() { return arm; }
    // /**
    //  * Get the left gripper
    //  * @return The left gripper servo.
    //  */
    // public Servo getGl() { return gl; }
    // /**
    //  * Get the right gripper
    //  * @return The right gripper servo.
    //  */
    // public Servo getGr() { return gr; }
    // /**
    //  * Get the left intake wrist
    //  * @return The left intake wrist servo.
    //  */
    // public Servo getIwl() { return iwl; }
    // /**
    //  * Get the right intake wrist
    //  * The right intake wrist servo.
    //  */
    // public Servo getIwr() { return iwr; }


    /**
     * Opens the gripper
     */
    // public void gripperOpen() {
    //     gl.setPosition(0.2);
    //     gr.setPosition(0.2);
    // }


    // /**
    //  * Closes the gripper
    //  */
    // public void gripperClose() {
    //     gl.setPosition(0.0);
    //     gr.setPosition(0.0);
    // }
}