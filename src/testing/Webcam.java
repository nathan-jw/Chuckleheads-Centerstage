package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp
public class Webcam extends LinearOpMode {
  OpenCvInternalCamera phoneCam;
  CamPipeline pipeline;
  

  @Override
  public void runOpMode() {
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
    pipeline = new CamPipeline();
    phoneCam.setPipeline(pipeline);

    //Optimize view so that we can get most accurate image
    phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

    //Start of Stream/Opening Camera
    phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
      @Override
      public void onOpened() {
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
      }

      @Override
      public void onError(int errorCode) {
        // for if camera unopened
      }
    });

    waitForStart();

    while (opModeIsActive()) {
      telemetry.addData("Analysis", pipeline.getAnalysis());
      telemetry.update();
      //prevent burn out cpu
      sleep(50);
    }
  }
}
