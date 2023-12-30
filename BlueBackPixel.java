package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueBackPixel")
public class BlueBackPixel extends LinearOpMode 
{
  // keep this constant, even though it is duplicated in AutoCommon  
  double DRIVE_POWER = 0.5;
 
 /**
   * This function is executed when this OpMode is selected & Init is pressed
   */
  @Override
  public void runOpMode() 
  {
    double xAdjustment = 0, yAdjustment = 0;  // default to no adjustments in case our tag isn't detected

    ElapsedTime programTime = new ElapsedTime();

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
      telemetry.addLine("Waiting for start");
      telemetry.addData("PROP LOCATION: ", lib.FindPropLocation());
      telemetry.addData("-","-");
      telemetry.addData(" -MAX_DELTA_CHROMA: ", lib.FindPropMaxDeltaChroma());
      telemetry.addData(" -MAX_CHROMA: ", lib.FindPropMaxChroma());
      telemetry.addData(" -MIN_CHROMA: ", lib.FindPropMinChroma());
      telemetry.addData(" -MAX_X: ", lib.FindPropMaxX());
      telemetry.addData(" -MAX_Y: ", lib.FindPropMaxY());
      telemetry.update();
    }

    lib.FindPropSetEnableDetection(false);
    lib.initAprilTag();
    
    // wait for user to press start on Driver Station
    waitForStart();
    programTime.reset();

    if (opModeIsActive()) {
      /************************************************************************
       * START / RUN OPMODE CODE:
       *   -perform autonomous driving based on identified prop location
       ************************************************************************/
      
      if (lib.FindPropLocation() == "LEFT"){
        /************************************************************************
         *
         * LEFT: (BLUE BACK)
         *
         ************************************************************************/
        lib.drive(17.5,0,0,DRIVE_POWER);        // Fwd 15" toward spike marks
        lib.drive(0,0,-50,DRIVE_POWER);         // CCW -45 deg to face LEFT spike mark

        lib.dropBottomPixel();

        lib.drive(-2.5, 0, 0, 0.2);       // back up: don't run over the pixel we just placed,
                                    // ...but also don't go back to far into frames
        lib.drive(0, 0, 50, DRIVE_POWER);      // CW 135 to face away from backdrop
        lib.drive(-2, -2, 0, DRIVE_POWER);
        lib.drive(0, 0, 90, DRIVE_POWER);      // CW 135 to face away from backdrop
        
        lib.armraise(100, 0.3);         // raise arm 120 deg (all the way back/up for placing pixel on board)
        lib.drive(-22, -5, 0, DRIVE_POWER);       // BACK 24" toward backdrop
        lib.reverseFlipper();           // put flipper in rev pos for placing pixel on board
        lib.armraisewait(30, 0.09);        // raise arm last 45 deg

        AprilTagDetection tagInfo = lib.getAprilTag_BlueLeft();
        yAdjustment = lib.getYAdjustmentForTag(tagInfo);
        xAdjustment = lib.getXAdjustmentForTag(tagInfo);

        lib.drive(-6.0 + yAdjustment, xAdjustment, 0, 0.2);       // REV last 4" to board
        lib.openClampWait();          // release pixel on board
        sleep(350);                // wait for pixel
        lib.openClampLittle();        // open clamp slightly
        lib.drive(3, 0, 0, DRIVE_POWER);  // get away from board
        lib.groundTransitionFlipper();  // get arm in position to clear axle
        lib.armraise(-100, 0.41);        // bring the arm back down
        lib.drive(0, 17, 0, DRIVE_POWER); // right 20" along backboard
        lib.armraise(-20.6, 0.221);      // lower arm back to ground to prevent slamming between programs
                                    // ...leave up ~15 deg from driving to park pos
        lib.armextend(0, 0.3);          // reset arm extension
        lib.drive(-13, 0, 0, DRIVE_POWER);  // back 13" into corner
        lib.armraise(0, 0.159);     // finish lowering claw to ground
        lib.normalFlipper();            // square w/ ground

      } else if (lib.FindPropLocation() == "MIDDLE") { // If pixel is in MIDDLE
        /************************************************************************
         *
         * MIDDLE: (BLUE BACK)
         *
         ************************************************************************/
        lib.drive(25.0,0,0,DRIVE_POWER);        // Fwd 15" toward spike marks
        lib.drive(0,0,-18,DRIVE_POWER);         // CCW 5 deg to be slightly offset from center of MIDDLE spike mark

        lib.dropBottomPixel();

        lib.drive(-4, 0, 0, 0.2);       // back up: don't run over the pixel we just placed,
                                    // ...but also don't go back to far into frames
        lib.drive(0, 0, 108, DRIVE_POWER);      // CW 108 to face away from backdrop
        sleep(50);
        lib.armraise(100, 0.3);         // raise arm 120 deg (all the way back/up for placing pixel on board)
        lib.drive(-23, 0, 0, DRIVE_POWER);       // FWD 25.5" toward backdrop
        lib.reverseFlipper();           // put flipper in rev pos for placing pixel on board
        lib.drive(0, -9, 0, DRIVE_POWER);       // left 10" along backdrop
        lib.armraise(30, 0.127);        // slow down to avoid tipping over

        AprilTagDetection tagInfo = lib.getAprilTag_BlueMiddle();
        yAdjustment = lib.getYAdjustmentForTag(tagInfo);
        xAdjustment = lib.getXAdjustmentForTag(tagInfo);

        lib.drive(-6 + yAdjustment, xAdjustment, 0, 0.2);       // REV last 5" to board
        sleep(300);
        lib.openClampWait();          // release pixel on board
        sleep(400);
        lib.drive(2, 0, 0, DRIVE_POWER); // drive forward 2" to get off wall
        lib.groundTransitionFlipper();  // get arm in position to clear axle
        lib.armraise(-100, 0.41);        // bring the arm back down
        lib.drive(0, 30, 0, DRIVE_POWER); // drive right 18" to wall
        lib.armraise(-20.6, 0.221);      // lower arm further, still leave up ~15 deg for driving
        lib.armextend(0, 0.3);
        lib.drive(-15, 0, 0, DRIVE_POWER); // drive back 5" behind/over line
        lib.armraisewait(0, 0.159);     // finish lowering claw to ground
        lib.normalFlipper();            // square w/ ground

      } else { // RIGHT code
        /************************************************************************
         *
         * RIGHT: (BLUE BACK)
         *
         ************************************************************************/
        lib.drive(17.5,0,0,DRIVE_POWER);        // Fwd 15" toward spike marks
        lib.drive(0,0,50,DRIVE_POWER);         // CW 45 deg to face RIGHT spike mark
        lib.drive(5.5, 0, 0, DRIVE_POWER);

        lib.dropBottomPixel();

        lib.drive(-5.5, 0, 0, DRIVE_POWER);
        lib.drive(-2, 0, 0, DRIVE_POWER); // back up 2" to clear spike mark
        lib.drive(0, 0, 40, DRIVE_POWER);      // CW 40 to face away from backdrop
        sleep(50);
        lib.drive(-21.2, 0, 0, DRIVE_POWER);       // FWD 26" toward backdrop
        lib.armraise(100, 0.3);         // raise arm 120 deg (all the way back/up for placing pixel on board)
        //drive(0, 0, 180, DRIVE_POWER);      // CW 180 deg to back into backdrop
        lib.reverseFlipper();           // put flipper in rev pos for placing pixel on board
        lib.drive(0, -18, 0, DRIVE_POWER);       // left 4" along backdrop
        lib.armraise(30, 0.127);        // slow down to avoid tipping over

        AprilTagDetection tagInfo = lib.getAprilTag_BlueRight();
        yAdjustment = lib.getYAdjustmentForTag(tagInfo);
        xAdjustment = lib.getXAdjustmentForTag(tagInfo);
        
        lib.drive(-6 + yAdjustment, xAdjustment, 0, 0.2);       // REV last 5" to board
        lib.openClampWait();          // release pixel on board
        sleep(400);
        lib.drive(2 , 0, 0, 0.25);       // forward 2" from board
        lib.armraise(-100, 0.41);        // bring the arm back down
        lib.drive(0, 36, 0, DRIVE_POWER); // drive right 24" to wall
        lib.groundTransitionFlipperWait();  // get arm in position to clear axle
        lib.armraise(-20.6, 0.221);      // lower arm back to ground to prevent slamming between programs
                                    // ...leave up ~15 deg from driving to park pos
        lib.closeClamp();
        lib.armextend(0, 0.4);
        lib.drive(-13, 0, 10, DRIVE_POWER); // drive back 5" behind/over line
        lib.openClamp();
        lib.armraisewait(0, 0.159);     // finish lowering claw to ground
        lib.normalFlipperWait();            // square w/ ground
      }

      telemetry.addLine("Opmode COMPLETE");
      telemetry.update();
      while (opModeIsActive()) {
        sleep(100); // this opMode doesn't use the boilerplate loop pattern, if this changes, be sure to remove the sleep
      }
    }
  }
}
