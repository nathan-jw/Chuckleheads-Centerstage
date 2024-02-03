package org.firstinspires.ftc.teamcode.centerstage;

import org.firstinspires.ftc.teamcode.lib.MecanumChassis;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;

/**
 * Initializes hardware variables. Makes it so that one function call is
 * required in our teleop and autonomous files to get hardware variables.
 * @author Nathan W.
 */
public class RobotHardware {
    /**
     * Mecanum chassis representing our drive motors, dead wheels, and imu
     */
    private MecanumChassis mecanumChassis = null;
    
    private DcMotorEx intake = null;
    
    private DcMotorEx intakeLiftMotor = null;
    
    private DcMotorEx hook = null;
    
    private DcMotorEx slide = null;
    
    private Servo droneLauncher = null;
    
    private Servo gripperLowerLeft = null;
    
    private Servo gripperLowerRight = null;
    
    private Servo gripperUpperLeft = null;
    
    private Servo gripperUpperRight = null;
    
    private Servo intakeLiftLeft = null;
    
    private Servo intakeLiftRight = null;


    /**
     * Initialize the hardware variables
     * @param hardwareMap HardwareMap from the Opmode
     */
    public RobotHardware(HardwareMap hardwareMap) {
        // Declare mecanum chassis
        mecanumChassis = new MecanumChassis(
                hardwareMap.get(DcMotorEx.class, "frontLeft"),    // front left drive motor
                hardwareMap.get(DcMotorEx.class, "backLeft"),     // front right drive motor
                hardwareMap.get(DcMotorEx.class, "frontRight"),   // back left drive motor
                hardwareMap.get(DcMotorEx.class, "backRight"),    // back right drive motor
                hardwareMap.get(DcMotorEx.class, "frontLeft"),    // horizontal odometery pod
                hardwareMap.get(DcMotorEx.class, "frontRight"),   // vertical odometrty pod
                hardwareMap.get(BNO055IMU.class, "imu"));         // IMU
                
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        slide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        droneLauncher = hardwareMap.get(Servo.class, "droneLauncher");
        droneLauncher.setPosition(0);
        
        gripperUpperLeft = hardwareMap.get(Servo.class, "gripperUpperLeft");
        gripperUpperRight = hardwareMap.get(Servo.class, "gripperUpperRight");
        gripperLowerLeft = hardwareMap.get(Servo.class, "gripperLowerLeft");
        gripperLowerRight = hardwareMap.get(Servo.class, "gripperLowerRight");
        gripperLowerRight.setDirection(Servo.Direction.REVERSE);
        gripperUpperLeft.setDirection(Servo.Direction.REVERSE);
        lowerGripper();
        
        intakeLiftMotor = hardwareMap.get(DcMotorEx.class, "intakeLiftMotor");
        intakeLiftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intakeLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        
        intakeLiftLeft = hardwareMap.get(Servo.class, "intakeLiftLeft");
        intakeLiftRight = hardwareMap.get(Servo.class, "intakeLiftRight");
        intakeLiftRight.setDirection(Servo.Direction.REVERSE);
        lowerIntake();
        
        hook = hardwareMap.get(DcMotorEx.class, "hook");
        hook.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hook.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    
    public void launchDrone() {
        droneLauncher.setPosition(1);
    }
    
    
    public void intake(double power) {
        intake.setPower(power);
    }
    
    
    public void slide(double power) {
        slide.setPower(power);
    }
    
    
    public void lowerGripper() {
        gripperLowerLeft.setPosition(0.05);
        gripperLowerRight.setPosition(0.05);
        gripperUpperLeft.setPosition(1);
        gripperUpperRight.setPosition(1);
    }
    
    
    public void raiseGripper() {
        gripperLowerLeft.setPosition(0.5);
        gripperLowerRight.setPosition(0.5);
        gripperUpperLeft.setPosition(0);
        gripperUpperRight.setPosition(0);
    }
    
    
    public void lowerIntake() {
        intakeLiftMotor.setTargetPosition(1800);
        intakeLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeLiftMotor.setVelocity(1500);
        
        intakeLiftLeft.setPosition(1);
        intakeLiftRight.setPosition(1);
    }
    
    
    public void raiseIntake() {
        intakeLiftMotor.setTargetPosition(795);
        intakeLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeLiftMotor.setVelocity(1500);
        
        intakeLiftLeft.setPosition(0.55);
        intakeLiftRight.setPosition(0.55);
    }
    
    
    public void hook(double power) {
        hook.setPower(power);
    }


    /**
     * Get the mecanum chassis.
     * @return The mecanum chassis.
     */
    public MecanumChassis getMecanumChassis() { return mecanumChassis; }
    
    public DcMotorEx getHook() { return hook; }
    
    public DcMotorEx getIntake() { return intake; }
    
    public DcMotorEx getIntakeLiftMotor() { return intakeLiftMotor; }
    
    public Servo getIntakeLiftLeft() { return intakeLiftLeft; }
    
    public Servo getIntakeLiftRight() { return intakeLiftRight; }
    
    public Servo getGripperLowerLeft() { return gripperLowerLeft; }
    
    public Servo getGripperLowerRight() { return gripperLowerRight; }
    
    public Servo getGripperUpperLeft() { return gripperUpperLeft; }
    
    public Servo getGripperUpperRight() { return gripperUpperRight; }
    
    public Servo getDroneLauncher() { return droneLauncher; }
}