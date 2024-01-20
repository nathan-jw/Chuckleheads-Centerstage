package org.firstinspires.ftc.teamcode.testing;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import org.firstinspires.ftc.teamcode.testing.TargetPosition;


class CamPipeline extends OpenCvPipeline {
    //color constants
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    //adjustable core values for region location and size when scanning
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(109, 98);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(181, 98);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(253, 98);
    static final int REGION_WIDTH = 20;
    static final int REGION_HEIGHT = 20;

    /*
     * Points which actually define the sample region rectangles, derived from above values
     *
     * Example of how points A and B work to define a rectangle
     *
     *   ------------------------------------
     *   | (0,0) Point A                    |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                  Point B (70,50) |
     *   ------------------------------------
     *
     */

    //Found this :P

    Point region1_pointA = new Point(
      REGION1_TOPLEFT_ANCHOR_POINT.x,
      REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
      REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
      REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region2_pointA = new Point(
      REGION2_TOPLEFT_ANCHOR_POINT.x,
      REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
      REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
      REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region3_pointA = new Point(
      REGION3_TOPLEFT_ANCHOR_POINT.x,
      REGION3_TOPLEFT_ANCHOR_POINT.y);
    Point region3_pointB = new Point(
      REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
      REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    //Variables for color detection
    Mat region1_Cb, region2_Cb, region3_Cb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    int avg1, avg2, avg3;

    private volatile TargetPosition position = TargetPosition.LEFT;

    /*
        convert from RGB to YCrCb because its more efficient
        extracts cb to be used
    */

    void inputToCb(Mat input) {
      Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
      Core.extractChannel(YCrCb, Cb, 1);
    }

    @Override
    public void init(Mat firstFrame) {

      //initializes cb

      inputToCb(firstFrame);

      //persistent reference to parent region so changes to this will effect parent
      //makes everything easier
      region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
      region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
      region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
    }

    @Override
    public Mat processFrame(Mat input) {

      /*
       * 
       *
       * chroma and
       * luma are intertwined. In YCrCb, chroma and luma are separated.
       * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
       * are Y, the luma channel (which essentially just a B&W image), the
       * Cr channel, which records the difference from red, and the Cb channel,
       * which records the difference from blue. Because chroma and luma are
       * not related in YCrCb, vision code written to look for certain values
       * in the Cr/Cb channels will not be severely affected by differing
       * light intensity, since that difference would most likely just be
       * reflected in the Y channel.
       *
       * after convert to YCrCb, we extract desired channel,
       *
       *
       * take the average pixel value of 3 different regions on that Cb
       * channel, one positioned over each stone. 
       *
       * We adraw rectangles on the screen showing where the sample regions
       * are, as well as drawing a solid rectangle over top the sample region
       *

       */

      //get Cb channel and its values
      inputToCb(input);

      //find average pixel value of each region
      avg1 = (int) Core.mean(region1_Cb).val[0];
      avg2 = (int) Core.mean(region2_Cb).val[0];
      avg3 = (int) Core.mean(region3_Cb).val[0];

      //visual aid
      Imgproc.rectangle(
        input, // Buffer to draw on
        region1_pointA, // First point which defines the rectangle
        region1_pointB, // Second point which defines the rectangle
        BLUE, // The color the rectangle is drawn in
        2); // Thickness of the rectangle lines

      Imgproc.rectangle(
        input,
        region2_pointA,
        region2_pointB,
        BLUE,
        2);

      Imgproc.rectangle(
        input,
        region3_pointA,
        region3_pointB,
        BLUE,
        2);

      //find max to help identify desired region
      int maxOneTwo = Math.max(avg1, avg2);
      int max = Math.max(maxOneTwo, avg3);

      //need to find desired region by using checks
      if (max == avg1) {
        position = TargetPosition.LEFT;

        Imgproc.rectangle(
          input,
          region1_pointA,
          region1_pointB,
          GREEN,
          -1);
      } else if (max == avg2) {
        position = TargetPosition.CENTER;

        //draw rectangle as visual aid
        Imgproc.rectangle(
          input, // Buffer to draw on
          region1_pointA, // First point which defines the rectangle
          region1_pointB, // Second point which defines the rectangle
          GREEN, // The color the rectangle is drawn in
          -1); // Negative thickness means solid fill
      } else if (max == avg3) {
        position = TargetPosition.RIGHT;

        Imgproc.rectangle(
          input,
          region3_pointA,
          region3_pointB,
          GREEN,
          -1);
      }
      //return our version of the feed to the viewport to use in scanning and viewing
      return input;
    }

    //get bot position for auton 
    public TargetPosition getAnalysis() {
      return position;
    }
  }