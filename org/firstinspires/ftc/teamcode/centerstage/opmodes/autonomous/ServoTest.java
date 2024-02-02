package org.firstinspires.ftc.teamcode.centerstage.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="ServoTest", group="Autonomous")
public class ServoTest extends LinearOpMode{
    private Servo served = null;
     @Override
    public void runOpMode() {
        served = hardwareMap.get(Servo.class, "1");
        waitForStart();
        
        while (opModeIsActive()){
            
            served.setPosition(0.0);
            
        }
        
        
        
    }
    
}