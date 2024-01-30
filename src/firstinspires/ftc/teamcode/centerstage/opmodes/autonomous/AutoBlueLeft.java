package org.firstinspires.ftc.teamcode.centerstage.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.centerstage.RobotHardware;
import org.firstinspires.ftc.teamcode.lib.AutonomousControl.Side;


@Autonomous(name="AutoBlueLeft", group="Autonomous")
public class AutoBlueLeft extends LinearOpMode {
    private RobotHardware rh = null;
    
    @Override
    public void runOpMode() {
        rh = new RobotHardware(hardwareMap);
        
        // rh.getMc().queueMoveTo(0, 12, 0.3);
        // rh.getMc().queueMoveTo(3, 0, 0.3);
        // rh.getMc().queueMoveTo(-3, 0, 0.3);
        
        rh.getMc().queueMoveTo(0, 12, 0.3);
        
        rh.getMc().queueTurnTo(3.14, 0.8, Side.RIGHT);
        
        rh.getMc().queueMoveTo(0, -12, 0.3);
        
        waitForStart();

        while (opModeIsActive()) {
            rh.getMc().runQueue();
            
            telemetry.addData("x", rh.getMc().getPos()[0]);
            telemetry.addData("y", rh.getMc().getPos()[1]);
            telemetry.addData("angle", rh.getMc().currentAngle());

            telemetry.update();
        }
    }
}