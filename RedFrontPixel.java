package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "RedFrontPixel")
public class RedFrontPixel extends LinearOpMode 
{
  double DRIVE_POWER = 0.5;

 /**
   * This function is executed when this OpMode is selected & Init is pressed
   */
  @Override
  public void runOpMode() 
  {
    double xAdjustment = 0, yAdjustment = 0;  // default to no adjustments in case our tag isn't detected

    // See FindPropVisInitData.java for calibration guide
    FindPropVisInitData visInitData = new FindPropVisInitData();
    visInitData.ColorChannel = 1;               // channel 1: red, channel 2: blue
    visInitData.MinDeltaDetectionChroma = 25;
    visInitData.ScanLowestYBlock = 4;
    visInitData.ScanLeftmostXBlock = 4;
    visInitData.MidRightXBoundary = 9;
    
    AutoCommon lib = new AutoCommon(
      hardwareMap,
      visInitData);

    // Pre-load 2 Pixels
    lib.preLoadPixels();

    while (!opModeIsActive() & !isStopRequested())
    {
      telemetry.addLine("Waiting for start");
      telemetry.addData("PROP LOCATION: ", lib.FindPropLocation());
      telemetry.addData("-","-");
      telemetry.addData(" -MAX_DELTA_CHROMA: ", lib.FindPropMaxDeltaChroma());
      telemetry.addData(" -MAX_CHROMA: ", lib.FindPropMaxChroma());
      telemetry.addData(" -MIN_CHROMA: ", lib.FindPropMinChroma());
      telemetry.addData(" -MAX_X: ", lib.FindPropMaxX());
      telemetry.addData(" -MAX_Y: ", lib.FindPropMaxY());
      telemetry.addData("Yaw", lib.getImuYaw());
      telemetry.update();
    }

    lib.FindPropSetEnableDetection(false);
    lib.initAprilTag();
    // wait for user to press start on Driver Station
    waitForStart();

    if (opModeIsActive()) {
      /************************************************************************
       * START / RUN OPMODE CODE:
       *   -perform autonomous driving based on identified prop location
       ************************************************************************/
      
      if (lib.FindPropLocation() == "LEFT"){
        /************************************************************************
         *
         * LEFT: (RED FRONT)
         *
         ************************************************************************/
        lib.drive(17.5, 0, 0,DRIVE_POWER);        // Fwd 15" toward spike marks
        lib.drive(0,0,-45,DRIVE_POWER);         // CCW -45 deg to face LEFT spike mark

        lib.dropBottomPixel();

        lib.drive(0, 6, 0, DRIVE_POWER); // Forward 15" to middle of field
        lib.drive(0, 0, 45, DRIVE_POWER); // Rotate 45 deg back to facing forward
        lib.drive(28, 0, 0, 0.65); // Forward 15" to middle of field
        double imuRotation = lib.getImuYaw();
        if (imuRotation != 0) {
          //telemetry.addData("imu yaw", imuRotation);
          //telemetry.addData("plan to turn", 90+imuRotation);
          //telemetry.update();
          //sleep(2000);
          lib.drive(0, 0, (90 + imuRotation), 0.65);      // Rotate CCW 90 dg to face back wall
        } else {
          lib.drive(0, 0, 90, 0);
        }
        lib.drive(69, 0, 0, 0.9); // Forward 5'
        imuRotation = lib.getImuYaw();
        if (imuRotation != 0.0) {
          //telemetry.addData("imu yaw", imuRotation);
          //telemetry.addData("plan to turn", imuRotation-90);
          //telemetry.update();
          //sleep(2000);
          lib.drive(0, 0, (imuRotation - 90), 0.65);             // rotate to back side
        } else {
          telemetry.addData("imu dead", 0.0);
          telemetry.update();
          sleep(2000);
          lib.drive(0, 0, -180, 0.65);
        }
        lib.drive(0, -24, 0, 0.65);
        lib.armraisewait(100,0.3);
        lib.reverseFlipper();
        lib.armraisewait(30,0.2);

        AprilTagDetection tagInfo = lib.getAprilTag_RedLeft();
        yAdjustment = lib.getYAdjustmentForTag(tagInfo);
        xAdjustment = lib.getXAdjustmentForTag(tagInfo);

        lib.drive(-6.0 + yAdjustment,-4.3 + xAdjustment,0,DRIVE_POWER);
        lib.openClampWait();
        lib.drive(1, 0, 0, 0.2);
        lib.normalFlipper();
        lib.armraise(0,0.2);
        lib.drive(-1.5, 0, 0, 0.4);     // drive to park
        
        // release motors?

      } else if (lib.FindPropLocation() == "MIDDLE") { // If pixel is in MIDDLE
        /************************************************************************
         *
         * MIDDLE: (RED FRONT)
         *
         ************************************************************************/
        lib.drive(25.5,-5,0,DRIVE_POWER);        // Fwd 15" toward spike marks

        lib.dropBottomPixel();

        lib.drive(-4, -10, 0, DRIVE_POWER);
        lib.drive(30, 0, 0, DRIVE_POWER);
        lib.drive(0, 0, 90, DRIVE_POWER);
        double imuRotation = lib.getImuYaw();
        if (imuRotation != 0) {
          lib.drive(0, 0, (91 + imuRotation), 0.65);      // Rotate CCW 90 dg to face back wall
        } else {
          lib.drive(0, 0, 89, 0);
        }
        lib.drive(85, 0, 0, 0.65);
        imuRotation = lib.getImuYaw();
        if (imuRotation != 0.0) {
          lib.drive(0, 0, (imuRotation - 91), 0.65);             // rotate to back side
        } else {
          lib.drive(0, 0, -180, 0.65);
        }
        lib.drive(0, -24, 0, 0.65);
        lib.armraisewait(100, 0.2);
        lib.reverseFlipper();
        lib.armraisewait(30,0.2);

        AprilTagDetection tagInfo = lib.getAprilTag_RedMiddle();
        yAdjustment = lib.getYAdjustmentForTag(tagInfo);
        xAdjustment = lib.getXAdjustmentForTag(tagInfo);

        lib.drive(-6 + yAdjustment,-4 + xAdjustment,0,DRIVE_POWER);
        lib.openClampWait();
        lib.drive(1, 0, 0, 0.2);
        lib.normalFlipper();
        lib.armraise(0,0.2);
        lib.drive(-1.5, 0, 0, 0.4);     // drive to park
        
      } else { // RIGHT code
        /************************************************************************
         *
         * RIGHT: (RED FRONT)
         *
         ************************************************************************/
  
        lib.drive(17.5,0,0,DRIVE_POWER);        // Fwd 15" toward spike 
        lib.drive(0,0,50,DRIVE_POWER);         // CW 45 deg to face RIGHT spike mark
        lib.drive(5.5, 0, 0, DRIVE_POWER);

        lib.dropBottomPixel();

        DRIVE_POWER = 0.65;
        lib.drive(-2, -7.5, 0, DRIVE_POWER); // Slide left 7" to go around pixel/marker
        lib.drive(0, 0, -50, DRIVE_POWER);
        lib.drive(25, 0, 0, DRIVE_POWER); // Forward 15" to middle of field
        double imuRotation = lib.getImuYaw();
        if (imuRotation != 0) {
          lib.drive(0, 0, (90 + imuRotation), 0.65);      // Rotate CCW 90 dg to face back wall
        } else {
          lib.drive(0, 0, 90, 0);
        }
        lib.drive(75.5, 0, 0, DRIVE_POWER); // Forward 5' 7"
        imuRotation = lib.getImuYaw();
        if (imuRotation != 0.0) {
          lib.drive(0, 0, (imuRotation - 90), 0.65);             // rotate to back side
        } else {
          lib.drive(0, 0, -180, 0.65);
        }
        lib.drive(0, -35, 0, DRIVE_POWER);
        lib.armraisewait(100,0.3);
        lib.reverseFlipper();
        lib.armraisewait(30,0.2);

        AprilTagDetection tagInfo = lib.getAprilTag_RedRight();
        yAdjustment = lib.getYAdjustmentForTag(tagInfo);
        xAdjustment = lib.getXAdjustmentForTag(tagInfo);

        lib.drive(-6.0 + yAdjustment, -1 + xAdjustment,0,0.5);
        lib.openClampWait();
        lib.drive(1, 0, 0, 0.2);
        lib.normalFlipper();
        lib.armraise(0,0.2);
        lib.drive(-1.5, 0, 0, 0.4);     // drive to park
      }

      telemetry.addLine("Opmode COMPLETE");
      telemetry.update();
      while (opModeIsActive()) {
        sleep(100); // this opMode doesn't use the boilerplate loop pattern, if this changes, be sure to remove the sleep
      }
    }
  }
}
