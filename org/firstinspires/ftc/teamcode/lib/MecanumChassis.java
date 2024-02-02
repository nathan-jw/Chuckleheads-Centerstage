package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;

import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.lib.math.MathUtils;
import org.firstinspires.ftc.teamcode.lib.AutonomousControl.Side;

/**
 * Represents a robot chassis with four mecanum wheels, two odometry pods, and an IMU.
 * @author Nathan W.
 */
public class MecanumChassis {
    /******************* PRIVATE VARIABLES *******************/
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
     * The amount of deviation allowed between the robot's actual and target positions in inches.
     */
    private double movementTolerance = 1.0;

    /**
     * The amount of deviation allowed between the robot's actual and target angles in radians.
     */
    private double angleTolerance = 0.3;

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
     * The target angle of the robot in radians
     */
    private double angleTarget = 0;

    /**
     * The robot's autonomous action queue.
     */
    private ArrayList<Queueable> queue = new ArrayList<Queueable>();


    /******************* PUBLIC FUNCTIONS *******************/
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
    public MecanumChassis(DcMotorEx fl, DcMotorEx bl, DcMotorEx fr, DcMotorEx br, DcMotorEx oph, DcMotorEx opv, BNO055IMU imu) {
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
     * Move the robot by a given x, y, turning power, as well as an overall max speed.
     * Mostly used in teleop. For autonomous, use moveTo(), or queueMoveTo() and turnTo() or queueTurnTo().
     * @param xPower movement power along x-axis.
     * @param yPower movemnet power along y-axis.
     * @param turningPower turning power. Increasing this makes the robot turn.
     * @param maxSpeed maximum maxSpeed the robot should move at. Arbitrary.
     * @see moveTo()
     * @see queueMoveTo()
     * @see turnTo()
     * @see queueTurnTo()
     */
    public void moveBy(double xPower, double yPower, double turningPower, double maxSpeed) {
        double theta = Math.atan2(yPower, xPower);
        double power = Math.hypot(xPower, yPower);
        power = MathUtils.clamp(-0.8, 0.8, power);

        theta += currentAngle();

        double sin = Math.sin(theta - Math.PI * 0.25);
        double cos = Math.cos(theta - Math.PI * 0.25);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double frontLeftPower = power * cos / max + turningPower;
        double frontRightPower = power * sin / max - turningPower;
        double backLeftPower = power * sin / max + turningPower;
        double backRightPower = power * cos / max - turningPower;

        fl.setPower(frontLeftPower * maxSpeed);
        bl.setPower(backLeftPower * maxSpeed);
        fr.setPower(frontRightPower * maxSpeed);
        br.setPower(backRightPower * maxSpeed);
    }


    /**
     * Attempts to move the robot to the given position in inches, field-centric.
     * Used in autonomous. queueMoveTo() is preferred over calling this directly.
     * @param targetX The x position in inches.
     * @param targetY The y position in inches.
     * @param maxSpeed The max speed to move at. Arbitrary.
     * @return Whether or not the distance between the robot's real and target positions is less than movementTolerance.
     * @see queueMoveTo()
     */
    public boolean moveTo(double targetX, double targetY, double maxSpeed) {
        updatePos();

        double xDiff = xInches - targetX;
        double yDiff = yInches - targetY;

        boolean atXTarget = Math.abs(xDiff) < movementTolerance;
        boolean atYTarget = Math.abs(yDiff) < movementTolerance;

        double xPower = 0.0;
        double yPower = 0.0;

        if (!atXTarget) {
            // TODO: adjust sqrt curves
            xPower = Math.sqrt(Math.abs(xDiff)) * xDiff < 0 ? 1.0 : -1.0;
        }
        if (!atYTarget) {
            yPower = Math.sqrt(Math.abs(yDiff)) * yDiff < 0 ? 1.0 : -1.0;
        }

        moveBy(xPower, yPower, 0, maxSpeed);

        return atXTarget && atYTarget;
    }
    
    
    /**
     * Add a moveTo command to the robot's queue.
     * @param targetX the x position in inches.
     * @param targetY the y position in inches.
     * @param maxSpeed The max speed to move at. Arbitrary.
     * @see moveTo()
     */
    public void queueMoveTo(double targetX, double targetY, double speed) {
        queue.add(new MoveToQueueable(targetX, targetY, speed));
    }


    /**
     * Attempt to turn the robot to the given angle target with the given max speed.
     * Will turn left or right depending on the value of side variable. queueTurnTo() is preferred over calling this directly.
     * @param angleTarget The angle in radians to turn the robot to.
     * @param maxSpeed The max speed at which to move the robot.
     * @param side The side to turn on (left or right).
     * @return Whether or not the robot is at the target angle with.
     * @see org.firstinspires.ftc.teamcode.lib.AutonomousControl.Side
     * @see queueTurnTo()
     */
    public boolean turnTo(double angleTarget, double maxSpeed, Side side) {
        updatePos();

        this.angleTarget = angleTarget;

        boolean atTarget = 
                Math.abs(angleTarget - currentAngle()) <= angleTolerance 
                || Math.abs(angleTarget + currentAngle()) <= angleTolerance;

        if (!atTarget) {
            if (side == Side.LEFT) { moveBy(0, 0, -0.5, maxSpeed); }
            else { moveBy(0, 0, 0.5, maxSpeed); }
        }
        else {
            imuInit();
            moveBy(0, 0, 0, 0);
            imuInit();
        }

        return atTarget;
    }
    
    
    /**
     * Add a turnTo command to the robot's queue.
     * @param angleTarget The angle in radians to turn the robot to.
     * @param maxSpeed The max speed at which to move the robot.
     * @param side The side to turn on (left or right).
     * @see turnTo()
     */
    public void queueTurnTo(double targetAngle, double speed, Side side) {
        queue.add(new TurnToQueueable(targetAngle, speed, side));
    }
    
    
    /**
     * Run the first task in the queue, removing it if the robot reaches the
     * task's target.
     */
    public void runQueue() {
        // TODO: add adjustable time padding between each task
        
        if (queue.size() == 0) { return; }
        if (queue.get(0).run()) {
            long start = System.currentTimeMillis();
            while (System.currentTimeMillis() < start + queue.get(0).timePadding) {}
            queue.remove(0);
        }
    }


    /**
     * Update the position of the robot in inches.
     */
    public void updatePos() {
        xTicks += oph.getCurrentPosition() - xTicks;
        yTicks += opv.getCurrentPosition() - yTicks;

        // Divide by amount of ticks per rotation
        xInches = (double) xTicks / 2000;
        yInches = (double) yTicks / 2000;

        // Mutliply by size of dead wheel in mm
        xInches *= 43;
        yInches *= 43;

        // Convert to inches
        xInches *= 0.03937;
        yInches *= 0.03937;
    }


    /**
     * Get the current position of the robot in inches.
     */
    public double[] getPos() { return new double[] {xInches, yInches}; }
    
    
    /**
     * Gets the current angle of the robot in radians
     * @return the current angle of the robot in radians
     */
    public double currentAngle() {
        return -imu.getAngularOrientation().firstAngle;
    }


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


    /******************* PUBLIC GETTERS *******************/
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
    
    public ArrayList<Queueable> getQueue() { return queue; }
    
    /**
    * Represents a queueable event for the robot, such as turning and movement commands.
    */
    interface Queueable {
        /**
         * The amount of milliseconds to wait after the Queueable reaches its target before it moves on.
         */
        double timePadding = 2000;
        
        /**
         * Perform the Queueable's action.
         * @return The Queueable's action result.
        */
        boolean run();
    }


    /**
     * Queueable for moving the robot to a specficic position, field-centric.
     */
    class MoveToQueueable implements Queueable {
        double targetX, targetY, maxSpeed;

        MoveToQueueable(double targetX, double targetY, double maxSpeed) {
            this.targetX = targetX;
            this.targetY = targetY;
            this.maxSpeed = maxSpeed;
        }

        @Override
        public boolean run() {
            return moveTo(targetX, targetY, maxSpeed);
        }
    }
    
    
    /**
     * Queueable for turning the robot to a specific angle.
     */
    class TurnToQueueable implements Queueable {
        double angleTarget, maxSpeed;
        Side side;

        TurnToQueueable(double angleTarget, double maxSpeed, Side side) {
            this.angleTarget = angleTarget;
            this.maxSpeed = maxSpeed;
            this.side = side;
        }

        @Override
        public boolean run() {
            return turnTo(angleTarget, maxSpeed, side);
        }
    }
}   
