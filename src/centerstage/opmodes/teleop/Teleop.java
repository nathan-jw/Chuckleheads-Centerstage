package org.firstinspires.ftc.teamcode.centerstage.opmodes.teleop;

// first
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.centerstage.RobotHardware;

// our static stuff


@TeleOp(name="Teleop", group="Linear Opmode")
public class Teleop extends LinearOpMode {
    private double moveGripperServo = 0.0;
    private boolean gunnerActive = false;

    RobotHardware rh = new RobotHardware(hardwareMap);

    @Override
    public void runOpMode() {
        // wait until init getIs() pressed
        waitForStart();

        while (opModeIsActive()) {
            setIsGunnerActive();
            move();
            ImuReinit();

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

        double rx = ((gamepad1.right_stick_y) * Math.sin(rh.getMc().currentAngle()) +
                (gamepad1.right_stick_x) * Math.cos(rh.getMc().currentAngle()));

        double maxSpeed = 0.7;
        if (gunnerActive) {
            maxSpeed /= 2;
        }

        rh.getMc().moveBy(x, y, rx, maxSpeed);
    }


    private void ImuReinit() {
        if (gamepad1.a) {
            rh.getMc().imuInit();
        }
    }
}
