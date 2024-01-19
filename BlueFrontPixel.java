package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "BlueFrontPixel")
public class BlueFrontPixel extends LinearOpMode 
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
    visInitData.ColorChannel = 2;               // channel 1: red, channel 2: blue
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
      if (gamepad1.y)
        lib.imuResetYaw();
      if (gamepad1.x)
        lib.imuReInit();
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
         * LEFT: (BLUE FRONT)
         *
         ************************************************************************/
        lib.drive(18, 0, 0, DRIVE_POWER);        // Fwd 4" to get motors off wall
        lib.drive(0,0,-55,DRIVE_POWER);         // CCW 55 deg to face LEFT spike mark
        lib.drive(7.5, 0, 0, DRIVE_POWER);        // drive forward to spike

        lib.dropBottomPixel();

        lib.drive(-7.5, 0, 0, DRIVE_POWER);        // drive back 5"
        lib.drive(0, 0, 55, DRIVE_POWER);      // CCW 55 to face forward
        lib.drive(31, 0, 0, 0.7);       // Drive 35" forward to bridge
        double imuRotation = lib.getImuYaw();
        if (imuRotation != 0) {
          lib.drive(0, 0, -(90 - imuRotation), 0.65);      // Rotate CCW 90 dg to face back wall
        } else {
          lib.drive(0, 0, -90, 0);
        }
        lib.drive(72, 0, 0, 0.75);      // Drive forward 6' 4" to parking zone, with second pixel
        imuRotation = lib.getImuYaw();
        if (imuRotation != 0.0) {
          lib.drive(0, 0, (imuRotation + 90), 0.65);             // rotate to back side
        } else {
          lib.drive(0, 0, 180, 0.65);
        }
        lib.drive(0, 32.2, 0, 0.75);    // line up with left side
        lib.armraisewait(100, 0.2);             // raise to backdrop
        lib.reverseFlipper();
        lib.armraisewait(30,0.2);               

        AprilTagDetection tagInfo = lib.getAprilTag_BlueLeft();
        yAdjustment = lib.getYAdjustmentForTag(tagInfo);
        xAdjustment = lib.getXAdjustmentForTag(tagInfo);

        lib.drive(-6.5 + yAdjustment, -.25 + xAdjustment, 0, DRIVE_POWER);       // go backwards to put on pixel
        lib.openClamp();
        lib.drive(1, 0, 0, DRIVE_POWER);
        lib.drive(3.5, 0, 0, DRIVE_POWER);      // go forwards
        lib.normalFlipper();
        lib.armraisewait(-70, 0.4);            // bring arm back down
        lib.armraise(-70,0.2);
        
        lib.drive(-8, -19, 0, 0.7);     // drive to park
        
        // release motors

      } else if (lib.FindPropLocation() == "MIDDLE") { // If pixel is in MIDDLE
        /************************************************************************
         *
         * MIDDLE: (BLUE FRONT)
         *
         ************************************************************************/
        lib.drive(26.5, 0, 0, DRIVE_POWER);        // Fwd 4" to get motors off wall

        lib.dropBottomPixel();

        lib.drive(-5, 0, 0, DRIVE_POWER);        // drive back 5"
        lib.drive(0, 13, 0, DRIVE_POWER);
        sleep(100);
        
        lib.drive(30, 0, 0, DRIVE_POWER);       // drive to bridge
        double imuRotation = lib.getImuYaw();
        if (imuRotation != 0) {
          lib.drive(0, 0, -(90 - imuRotation), 0.65);      // Rotate CCW 90 dg to face back wall
        } else {
          lib.drive(0, 0, -90, 0);
        }
        lib.drive(82, 0, 0, 0.9);       // Drive forward 83" to parking zone, with second pixel
        imuRotation = lib.getImuYaw();
        if (imuRotation != 0.0) {
          lib.drive(0, 0, (imuRotation + 90), 0.65);             // rotate to back side
        } else {
          lib.drive(0, 0, 180, 0.65);
        }
        lib.drive(0, 28, 0, DRIVE_POWER);    // line up with middle
        lib.armraisewait(100, 0.2);             // raise to backdrop
        lib.reverseFlipper();
        lib.armraisewait(30,0.2);               

        AprilTagDetection tagInfo = lib.getAprilTag_BlueMiddle();
        yAdjustment = lib.getYAdjustmentForTag(tagInfo);
        xAdjustment = lib.getXAdjustmentForTag(tagInfo);

        lib.drive(-6 + yAdjustment, -3 + xAdjustment, 0, DRIVE_POWER);       // go backwards to put on pixel
        lib.openClamp();
        lib.drive(1, 0, 0, 0.2);
        lib.drive(3, 0, 0, DRIVE_POWER);      // go forwards
        lib.normalFlipper();
        lib.armraisewait(-110, 0.4);            // bring arm back down
        lib.armraise(-30,0.2);
        
        lib.drive(-8, -16, 0, DRIVE_POWER);     // drive to park

      } else { // RIGHT code
        /************************************************************************
         *
         * RIGHT: (BLUE FRONT)
         *
         ************************************************************************/
        lib.drive(18, 0, 0, DRIVE_POWER);        // Fwd 4" to get motors off wall
        lib.drive(0,0,35,DRIVE_POWER);         // CCW 35 deg to face LEFT spike mark

        lib.dropBottomPixel();

        lib.drive(-4, -3, -35, DRIVE_POWER);      // CCW 35 to face forward
        lib.drive(33.5, 0, 0, 0.65);       // Drive 35" forward to bridge
        double imuRotation = lib.getImuYaw();
        if (imuRotation != 0) {
          lib.drive(0, 0, -(90 - imuRotation), 0.65);      // Rotate CCW 90 dg to face back wall
        } else {
          lib.drive(0, 0, -90, 0);
        }
        lib.drive(70, 0, 0, 0.65);      // Drive forward 6' 4" to parking zone, with second pixel
        imuRotation = lib.getImuYaw();
        if (imuRotation != 0.0) {
          lib.drive(0, 0, (imuRotation + 90), 0.65);  // rotate to back side
        } else {
          lib.drive(0, 0, 180, 0.65);
        }
        lib.drive(0, 16, 0, DRIVE_POWER);    // line up to right side
        lib.armraisewait(100, 0.2);           // lift arm to backdrop
        lib.reverseFlipper();
        lib.armraisewait(30,0.2);             

        AprilTagDetection tagInfo = lib.getAprilTag_BlueRight();
        yAdjustment = lib.getYAdjustmentForTag(tagInfo);
        xAdjustment = lib.getXAdjustmentForTag(tagInfo);

        lib.drive(-6 + yAdjustment, -3 + xAdjustment, 0, DRIVE_POWER);     // drive back to backdrop
        lib.openClamp();
        lib.drive(4.5, 0, 0, DRIVE_POWER);    // drive forward to let pixel drop
        lib.normalFlipper();
        lib.armraisewait(-110, 0.4);          // bring arm back down
        lib.armraise(-30,0.2);                
        
        lib.drive(-10, -10, 0, DRIVE_POWER);   // park
      }
      
      telemetry.addLine("Opmode COMPLETE");
      telemetry.update();
      while (opModeIsActive()) {
        sleep(100); // this opMode doesn't use the boilerplate loop pattern, if this changes, be sure to remove the sleep
      }
    }
  }
}
