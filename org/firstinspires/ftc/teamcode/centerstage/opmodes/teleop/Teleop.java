package org.firstinspires.ftc.teamcode.centerstage.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.centerstage.RobotHardware;
import org.firstinspires.ftc.teamcode.lib.MecanumChassis;

@TeleOp(name="Teleop", group="Linear Opmode")
public class Teleop extends LinearOpMode {
    private double moveGripperServo = 0.0;
    private boolean gunnerActive = false;

    private RobotHardware rh = null;
    private MecanumChassis mc = null;

    @Override
    public void runOpMode() {
        rh = new RobotHardware(hardwareMap);
        mc = rh.getMecanumChassis();
        
        // wait until init getIs() pressed
        waitForStart();

        while (opModeIsActive()) {
            setIsGunnerActive();
            
            move();
            ImuReinit();
            launchDrone();
            intake();
            slide();
            gripper();
            hook();

            telemetry.update();
        }
    }


    private void setIsGunnerActive() {
        // if any button on the gunner's controller getIs() pressed, set gunnerActive
        // to true. Otherwise, false.
        gunnerActive = !gamepad2.atRest() || gamepad2.dpad_up || gamepad2.dpad_down
                || gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.a
                || gamepad2.a || gamepad2.b || gamepad2.x || gamepad2.y
                || gamepad2.guide || gamepad2.start || gamepad2.back
                || gamepad2.left_bumper || gamepad2.right_bumper;
    }


    private void move() {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;

        double rx = ((gamepad1.right_stick_y) * Math.sin(mc.currentAngle()) +
                (gamepad1.right_stick_x) * Math.cos(mc.currentAngle()));

        double maxSpeed = 0.7; // fix: rename. Named maxSpeed, but controls power?
        if (gunnerActive) {
            maxSpeed /= 2;
        }

        mc.moveBy(x, y, rx, maxSpeed);
    }


    private void ImuReinit() {
        if (gamepad1.a) {
            mc.imuInit();
        }
    }
    
    
    private void launchDrone() {
        if (gamepad2.start) {
            rh.launchDrone();
        }
    }
    
    
    private void intake() {
        rh.intake(gamepad2.left_trigger - gamepad2.right_trigger);
    }
    
    
    private void hook() {
        double power = (gamepad2.dpad_left ? 1 : 0) - (gamepad2.dpad_right ? 1 : 0);
        rh.hook(power);
    }
    
    
    private void slide() {
        double power = (gamepad2.left_bumper ? 1 : 0) - (gamepad2.right_bumper ? 1 : 0);
        rh.slide(power);
    }
    
    
    private void gripper() {
        if (gamepad2.dpad_down) {
            rh.lowerGripper();
            rh.raiseIntake();
        }
        if (gamepad2.dpad_up) {
            rh.raiseGripper();
            rh.lowerIntake();
        }
    }
}