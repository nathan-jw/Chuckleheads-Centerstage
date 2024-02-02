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
                
        droneLauncher = hardwareMap.get(Servo.class, "droneLauncher");
        droneLauncher.setPosition(0);
        
        gripperUpperLeft = hardwareMap.get(Servo.class, "gripperUpperLeft");
        gripperUpperRight = hardwareMap.get(Servo.class, "gripperUpperRight");
        gripperLowerLeft = hardwareMap.get(Servo.class, "gripperLowerLeft");
        gripperLowerRight = hardwareMap.get(Servo.class, "gripperLowerRight");
        gripperLowerRight.setDirection(Servo.Direction.REVERSE);
        gripperUpperRight.setDirection(Servo.Direction.REVERSE);
        lowerGripper();
        
        intakeLiftMotor = hardwareMap.get(DcMotorEx.class, "intakeLiftMotor");
        intakeLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        
        intakeLiftLeft = hardwareMap.get(Servo.class, "intakeLiftLeft");
        intakeLiftRight = hardwareMap.get(Servo.class, "intakeLiftRight");
        intakeLiftRight.setDirection(Servo.Direction.REVERSE);
        lowerIntake();
    }
    
    
    public void launchDrone() {
        droneLauncher.setPosition(1);
    }
    
    
    public void intake(double power) {
        intake.setPower(power);
    }
    
    
    public void lowerGripper() {
        gripperLowerLeft.setPosition(0.025);
        gripperLowerRight.setPosition(0.025);
        // gripperUpperLeft.setPosition(0);
        // gripperUpperRight.setPosition(1.0);
    }
    
    
    public void raiseGripper() {
        gripperLowerLeft.setPosition(0.5);
        gripperLowerRight.setPosition(0.5);
        // gripperUpperLeft.setPosition(0);
        // gripperUpperRight.setPosition(0);
    }
    
    
    public void lowerIntake() {
        // intakeLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        // intakeLiftMotor.setTargetPosition(1000);
        
        intakeLiftLeft.setPosition(1);
        intakeLiftRight.setPosition(1);
    }
    
    
    public void raiseIntake() {
        // intakeLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        // intakeLiftMotor.setTargetPosition(1000);
        
        intakeLiftLeft.setPosition(0.5);
        intakeLiftRight.setPosition(0.5);
    }


    /**
     * Get the mecanum chassis.
     * @return The mecanum chassis.
     */
    public MecanumChassis getMecanumChassis() { return mecanumChassis; }
    
    public Servo getGripperLowerLeft() { return gripperLowerLeft; }
    
    public Servo getGripperLowerRight() { return gripperLowerRight; }
}