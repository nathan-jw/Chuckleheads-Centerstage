package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.lib.math.MathUtils;


/**
 * Represents a robot chassis with four mecanum wheels, two odometry pods, and an IMU.
 * @author Nathan W.
 */
public class MecanumChassis {
    /**
     * front left drive motor
     */
    private DcMotorEx fl = null;
    /**
     * back left drive motor
     */
    private DcMotorEx bl = null;
    /**
     * front right drive motor
     */
    private DcMotorEx fr = null;
    /**
     * back right drive motor
     */
    private DcMotorEx br = null;

    /**
     * horizontal dead wheel
     */
    private DcMotorEx oph = null;
    /**
     * vertical dead wheel
     */
    private DcMotorEx opv = null;

    /**
     * IMU
     */
    private BNO055IMU imu = null;

    /**
     * The amount of deviation allowed between the robot's real and target positions in inches.
     */
    private double movementTolerance = 3.0;

    /**
     * The amount of deviation allowed between the robot's real and target angles in radians.
     */
    private double angleTolerance = 0.2;

    /**
     * The current x position of the robot in inches
     */
    private double xInches = 0.0;
    /**
     * The current y position of the robot in inches
     */
    private double yInches = 0.0;

    /**
     * The current x position of the robot in ticks
     */
    private int xTicks = 0;

    /**
     * The current y position of the robot in ticks
     */
    private int yTicks = 0;


    /**
     * Initialize the motor variables
     * @param fl front left drive motor
     * @param bl back left drive motor
     * @param fr front right drive motor
     * @param br back right drive motor
     * @param oph horizontal odometry pod
     * @param opv vertical odometry pod
     * @param imu robot's IMU
     */
    public MecanumChassis(DcMotorEx fl, DcMotorEx bl, DcMotorEx fr,
                          DcMotorEx br, DcMotorEx oph, DcMotorEx opv, BNO055IMU imu) {

        this.fl = fl;
        this.bl = bl;
        this.fr = fr;
        this.br = br;

        this.oph = oph;
        this.opv = opv;

        this.imu = imu;

        fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        oph.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        opv.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // set modes
        fl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        fl.setDirection(DcMotorEx.Direction.REVERSE);
        bl.setDirection(DcMotorEx.Direction.REVERSE);

        imuInit();
    }


    /**
     * Move the robot.
     * @param x movement power along x axis.
     * @param y movemnet power along y axis.
     * @param rx turning power. Increasing this makes the robot turn while moving
     * @param maxSpeed maximum speed the robot should move at.
     */
    public void moveBy(double x, double y, double rx, double maxSpeed) {
        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);
        power = MathUtils.clamp(-0.8, 0.8, power);

        theta += currentAngle();

        double sin = Math.sin(theta - Math.PI * 0.25);
        double cos = Math.cos(theta - Math.PI * 0.25);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double frontLeftPower = power * cos / max + rx;
        double frontRightPower = power * sin / max - rx;
        double backLeftPower = power * sin / max + rx;
        double backRightPower = power * cos / max - rx;

        fl.setPower(frontLeftPower * maxSpeed);
        bl.setPower(backLeftPower * maxSpeed);
        fr.setPower(frontRightPower * maxSpeed);
        br.setPower(backRightPower * maxSpeed);
    }


    /**
     * attempts to move the robot to the given position in inches.
     * @param targetX the x position in inches that you want to robot to move to
     * @param targetY the y position in inches that you want the robot to move to
     * @return whether or not the current position is less than movement tolerance
     */
    public boolean moveTo(double targetX, double targetY, double rx, double maxSpeed) {
        boolean atXTarget = Math.abs(targetX - xInches) < movementTolerance;
        boolean atYTarget = Math.abs(targetY - yInches) < movementTolerance;

        double xPower = 0.0;
        double yPower = 0.0;

        if (!atXTarget) {
            xPower = 10.0;
        }
        if (!atYTarget) {
            yPower = 10.0;
        }

        moveBy(xPower, yPower, rx, maxSpeed);

        return atXTarget && atYTarget;
    }


    /**
     * Update the position of the robot in inches.
     */
    public void updatePos() {
        xTicks += oph.getCurrentPosition() - xTicks;
        yTicks += opv.getCurrentPosition() - yTicks;

        // Divide by amount of ticks per rotation
        xInches = (double) yTicks / 2000;
        yInches = (double) yTicks / 2000;

        // Mutliply by size of dead wheel in mm
        xInches *= 43;
        yInches *= 43;

        // Convert to inches
        xInches *= 0.03937;
        yInches *= 0.03937;

        /*
        // math
        Arc Length = radian * radius;

        // Code
        translationTicks = deltaTicks - (deltaTheta * radius)
        */
    }


    /**
     * Get the position of the robot in inches.
     */
    public double[] getPos() { return new double[] {xInches, yInches}; }


    /**
     * Initialize the IMU. Teleop requires we be able to do this on demand,
     * hence making it a seperate function. Will fail if imuInit() is called before hardwareInit()
     */
    public void imuInit() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
    }


    /**
     * Gets the current angle of the robot in radians
     * @return the current angle of the robot in radians
     */
    public double currentAngle() {
        return -imu.getAngularOrientation().firstAngle;
    }


    /**
     * get the front left drive motor
     * @return front left drive motor
     */
    public DcMotorEx getFl() { return fl; }
    /**
     * get the back left drive motor
     * @return back left drive motor
     */
    public DcMotorEx getBl() { return bl; }
    /**
     * get the front right drive motor
     * @return front right drive motor
     */
    public DcMotorEx getFr() { return fr; }
    /**
     * get the back right drive motor
     * @return back right drive motor
     */
    public DcMotorEx getBr() { return br; }
    /**
     * get the horizontal odometry pod
     * @return the horizontal odometry pod
     */
    public DcMotorEx getOph() { return oph; }
    /**
     * the vertical odometry pod
     * @return the vertical odometry pod
     */
    public DcMotorEx getOpv() { return opv; }
    /**
     * get the control hub's IMU
     * @return the control hub's IMU
     */
    public BNO055IMU getImu() { return imu; }
}