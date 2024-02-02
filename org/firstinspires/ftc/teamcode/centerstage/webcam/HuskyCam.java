// package org.firstinspires.ftc.teamcode.HuskyLens;

// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.hardware.dfrobot.HuskyLens;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.util.ElapsedTime;
// import com.qualcomm.robotcore.util.Range;
// import org.firstinspires.ftc.teamcode.HuskyLens.*;

// import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// import org.firstinspires.ftc.robotcore.internal.system.Deadline;

// import java.util.concurrent.TimeUnit;
// @TeleOp



// public class HuskyCam extends LinearOpMode {

//     private final int READ_PERIOD = 1;

//     private HuskyLens husky;

//     @Override
//     public void runOpMode()
//     {
//         husky = hardwareMap.get(HuskyLens.class, "huskyLens");

//         /*
//          * This sample rate limits the reads solely to allow a user time to observe
//          * what is happening on the Driver Station telemetry.  Typical applications
//          * would not likely rate limit.
//          */


//         /*
//          * Basic check to see if the device is alive and communicating.  This is not
//          * technically necessary here as the HuskyLens class does this in its
//          * doInitialization() method which is called when the device is pulled out of
//          * the hardware map.  However, sometimes it's unclear why a device reports as
//          * failing on initialization.  In the case of this device, it's because the
//          * call to knock() failed.
//          */


//         /*
//          * The device uses the concept of an algorithm to determine what types of
//          * objects it will look for and/or what mode it is in.  The algorithm may be
//          * selected using the scroll wheel on the device, or via software as shown in
//          * the call to selectAlgorithm().
//          *
//          * The SDK itself does not assume that the user wants a particular algorithm on
//          * startup, and hence does not set an algorithm.
//          *
//          * Users, should, in general, explicitly choose the algorithm they want to use
//          * within the OpMode by calling selectAlgorithm() and passing it one of the values
//          * found in the enumeration HuskyLens.Algorithm.
//          */
//         husky.cmdRequestAlgorithm(husky.COLOR_RECOGNITION);

//         telemetry.update();
//         waitForStart();

//         /*
//          * Looking for AprilTags per the call to selectAlgorithm() above.  A handy grid
//          * for testing may be found at https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336#target_20.
//          *
//          * Note again that the device only recognizes the 36h11 family of tags out of the box.
//          */
//         while(opModeIsActive()) {

//                         husky.cmdRequest(husky.BLOCKS);
//             int l = husky.blocks.length;
//             telemetry.addData("blocks", l);
//             for (int i = 0; i < l; i++) {
//                 telemetry.addData("block" + i, husky.blocks[i].id());
//             }
// }}
//     // private int checkColor(){
//     //     if
//     // }
// }