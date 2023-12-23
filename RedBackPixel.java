package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RedBackPixel")
public class RedBackPixel extends LinearOpMode 
{
  double DRIVE_POWER = 0.5;

 /**
   * This function is executed when this OpMode is selected & Init is pressed
   */
  @Override
  public void runOpMode() 
  {
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
      telemetry.update();
    }

    lib.FindPropSetEnableDetection(false);

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
         * LEFT: (RED BACK)
         *
         ************************************************************************/
        lib.drive(21,0,0,DRIVE_POWER);        // Fwd 15" toward spike marks
        lib.drive(0,0,-75,DRIVE_POWER);       // CCW 45 deg to face LEFT spike mark
        lib.drive(4,0,0,DRIVE_POWER);         // small adjustment toward spike mark

        lib.dropBottomPixel();                // extends 5", drop 1/re-grab 1, retract, raise 16 deg

        lib.drive(-4,0,0,DRIVE_POWER);        // back slightly away
        lib.drive(0, 0, (-15), DRIVE_POWER);  // CCW to face away from backdrop
        lib.armraise(100, 0.3);               // raise arm (quickly / most of way for placing pixel on board)
        lib.drive(-22.5, 0, 0, DRIVE_POWER);    // BACK toward backdrop
        lib.drive(0, 10.5, 0, DRIVE_POWER);   // Right to left side of backdrop
        lib.reverseFlipper();                 // put flipper in rev pos for placing pixel on board
        lib.armraisewait(30, 0.09);           // slow down to avoid tipping over

        lib.drive(-2.5, 0, 0, 0.2);             // REV last 5" to board
        lib.openClampWait();                  // release pixel on board
        sleep(350);
        lib.drive(3, 0, 0, DRIVE_POWER);      // Get off of board
        lib.normalFlipper();
        lib.armraise(-110, 0.4);              // bring the arm back down

        lib.drive(0, -26, 0, DRIVE_POWER);    // Drive left 26" to wall
        lib.armraise(-20.6, 0.221);           // lower arm back to ground to prevent slamming between programs
                                          // ...leave up ~15 deg from driving to park pos
        lib.drive(-10, 0, 0, DRIVE_POWER);    // Park over/behind line
        lib.armraise(0, 0.159);               // finish lowering claw to ground

      } else if (lib.FindPropLocation() == "MIDDLE") { // If pixel is in MIDDLE
        /************************************************************************
         *
         * MIDDLE: (RED BACK)
         *
         ************************************************************************/
        lib.drive(26.0,0,0,DRIVE_POWER);        // Fwd 15" toward spike marks

        lib.dropBottomPixel();                  // extends 5", drop 1/re-grab 1, retract, raise 16 deg
        
        // After drop-off
        lib.drive(-4, 0, 0, 0.2);               // back up: don't run over the pixel we just placed,
                                            // ...but also don't go back to far into frames
        sleep(100);
        lib.drive(0, 0, -90, DRIVE_POWER);      // CCW 83 to face away from backdrop    
        sleep(50);
        lib.armraise(100, 0.3);                 // raise arm 120 deg (all the way back/up for placing pixel on board)
        lib.drive(-21.5, 0, 0, DRIVE_POWER);    // BACK 25.5" toward backdrop
        lib.reverseFlipper();                   // put flipper in rev pos for placing pixel on board
        lib.drive(0, 4, 0, DRIVE_POWER);        // right 4" along backdrop to left pos
        lib.armraisewait(30, 0.127);            // slowly finish putting arm back
        lib.drive(-5, 0, 0, 0.2);               // REV last 5" to board
        // Place pixel on backdrop
        lib.openClampLittleWait();              // release pixel on board
        sleep(300);
        lib.openClamp();                        // fully open grabber
        lib.drive(2, 0, 0, DRIVE_POWER);        // forward 2" to get off board
        lib.normalFlipper();                    // get arm in position to clear axle
        lib.armraise(-110, 0.31);               // bring the arm back down
        lib.drive(0, -21, 0, DRIVE_POWER);      // drive right 18" to wall
        lib.armraise(-30, 0.31);                // lower arm back to ground to prevent slamming
        lib.drive(-13, 0, 0, DRIVE_POWER);      // drive back 5" behind/over line
        lib.armraisewait(0, 0.159);             // finish lowering claw to ground
        lib.normalFlipperWait();

      } else { // RIGHT code
        /************************************************************************
         *
         * RIGHT: (RED BACK)
         *
         ************************************************************************/
        lib.drive(17.5, 0, 0, DRIVE_POWER);   // Fwd 4" to get motors off wall
        lib.drive(0,0,35,DRIVE_POWER);        // CW 45 deg to face RIGHT spike mark
        lib.drive(1, 0, 0, 0.2);              // Fwd 1" - bump a little more toward the spike mark

        lib.dropBottomPixel();                // extends 5", drop 1/re-grab 1, retract, raise 16 deg
        
        lib.drive(-3, 0, 0, DRIVE_POWER);     // back up 3"
        lib.drive(0, 0, -125, DRIVE_POWER);   // CCW 125 deg to face away from backdrop
        lib.armraise(100, 0.3);               // raise arm 120 deg (all the way back/up for placing pixel on board)
        lib.drive(-25, 0, 0, DRIVE_POWER);    // Back 30" toward backdrop
        lib.reverseFlipper();                 // put flipper in rev pos for placing pixel on board
        lib.drive(0, 2, 0, DRIVE_POWER);      // Slide right 1"
        lib.armraise(30, 0.127);              // slow down to avoid tipping over
        lib.drive(-4.5, 0, 0, 0.2);           // REV last 5" to board
      // Place pixel on backdrop
        lib.openClampLittleWait();            // release pixel on board
        sleep(300);
        lib.openClamp();                      // Fully open grabber
        lib.drive(2, 0, 0, DRIVE_POWER);      // forward 2 to get away from backdrop
        lib.normalFlipperWait();              // get arm in position to clear axle
        lib.armraise(-110, 0.31);             // bring the arm back down
        lib.drive(0, -14, 0, DRIVE_POWER);    // drive left 8" to wall
        lib.armraise(-30, 0.221);             // lower arm further, leave up ~15 deg for driving
        lib.drive(-14, 0, 0, DRIVE_POWER);    // drive back 5" behind/over line
        lib.armraise(0, 0.159);               // finish lowering claw to ground
        lib.normalFlipperWait();              // square w/ ground
      }

      telemetry.addLine("Opmode COMPLETE");
      telemetry.update();
      while (opModeIsActive()) {
        sleep(100); // this opMode doesn't use the boilerplate loop pattern, if this changes, be sure to remove the sleep
      }
    }
  }
}
