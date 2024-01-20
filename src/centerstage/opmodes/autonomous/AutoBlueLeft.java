package org.firstinspires.ftc.teamcode.centerstage.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.centerstage.RobotHardware;


@Autonomous(name="AutoBlueLeft", group="Autonomous")
public class AutoBlueLeft extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware rh = new RobotHardware(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            rh.getMc().updatePos();

            rh.getMc().moveTo(0, 12, 0, 25);

            telemetry.addData("x", rh.getMc().getPos()[0]);
            telemetry.addData("y", rh.getMc().getPos()[1]);

            telemetry.update();
        }
    }
}