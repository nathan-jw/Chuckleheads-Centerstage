// package org.firstinspires.ftc.teamcode.webcam;

// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.HardwareMap;

// import org.openftc.easyopencv.OpenCvInternalCamera;
// import org.openftc.easyopencv.OpenCvPipeline;
// import org.openftc.easyopencv.OpenCvCameraFactory;
// import org.openftc.easyopencv.OpenCvCamera;
// import org.openftc.easyopencv.OpenCvCameraRotation;

// public class Webcam {
//   OpenCvInternalCamera phoneCam;
//   CameraPipeline pipeline;
  
//   public Webcam(HardwareMap hardwareMap) {
//     int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//     phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
//     pipeline = new CameraPipeline();
//     phoneCam.setPipeline(pipeline);

//     //Optimize view so that we can get most accurate image
//     phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

//     //Start of Stream/Opening Camera
//     phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//       @Override
//       public void onOpened() {
//         phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
//       }
      
//       @Override
//       public void onError(int errorCode) {
//         // for if camera unopened
//       }
//     });
//   }
// }