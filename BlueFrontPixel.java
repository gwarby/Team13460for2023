package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

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
        lib.drive(29.5, 0, 0, 0.65);       // Drive 35" forward to bridge
        lib.drive(0, 0, -90, 0.65);      // Rotate CCW 90 dg to face back wall
        lib.drive(75, 0, 0, 0.65);      // Drive forward 6' 4" to parking zone, with second pixel
        lib.drive(0, -32.2, 0, DRIVE_POWER);    // line up with left side
        lib.drive(0, 0, 180, 0.65);             // rotate to back side
        lib.armraisewait(100, 0.2);             // raise to backdrop
        lib.reverseFlipper();
        lib.armraisewait(30,0.2);               
        lib.drive(-6, 0, 0, DRIVE_POWER);       // go backwards to put on pixel
        lib.openClamp();
        lib.drive(4.5, 0, 0, DRIVE_POWER);      // go forwards
        lib.normalFlipper();
        lib.armraisewait(-110, 0.4);            // bring arm back down
        lib.armraise(-30,0.2);
        
        lib.drive(-3, -19, 0, DRIVE_POWER);     // drive to park
        lib.drive(-5, 0, 0, DRIVE_POWER);     // drive to park
        
        // release motors

      } else if (lib.FindPropLocation() == "MIDDLE") { // If pixel is in MIDDLE
        /************************************************************************
         *
         * MIDDLE: (BLUE FRONT)
         *
         ************************************************************************/
        lib.drive(20, 0, 0, DRIVE_POWER);        // Fwd 4" to get motors off wall
        lib.drive(5, 0, 0, DRIVE_POWER);        // drive forward to spike

        lib.dropBottomPixel();

        lib.drive(-5, 0, 0, DRIVE_POWER);        // drive back 5"
        lib.drive(0, 13, 0, DRIVE_POWER);
        sleep(100);
        
        lib.drive(31, 0, 0, DRIVE_POWER);       // drive to bridge
        sleep(100);
        lib.drive(0, 0, -90, 0.65);      // Rotate CCW 90 dg to face back wall
        lib.drive(88, 0, 0, 0.65);       // Drive forward 83" to parking zone, with second pixel
        lib.drive(0, -28, 0, DRIVE_POWER);    // line up with middle
        lib.drive(0, 0, 180, 0.65);             // rotate to back side
        lib.armraisewait(100, 0.2);             // raise to backdrop
        lib.reverseFlipper();
        lib.armraisewait(30,0.2);               
        lib.drive(-6, 0, 0, DRIVE_POWER);       // go backwards to put on pixel
        lib.openClamp();
        lib.drive(4.5, 0, 0, DRIVE_POWER);      // go forwards
        lib.normalFlipper();
        lib.armraisewait(-110, 0.4);            // bring arm back down
        lib.armraise(-30,0.2);
        
        lib.drive(-3, -16, 0, DRIVE_POWER);     // drive to park
        lib.drive(-7, 0, 0, DRIVE_POWER);     // drive to park

      } else { // RIGHT code
        /************************************************************************
         *
         * RIGHT: (BLUE FRONT)
         *
         ************************************************************************/
        lib.drive(18, 0, 0, DRIVE_POWER);        // Fwd 4" to get motors off wall
        lib.drive(0,0,35,DRIVE_POWER);         // CCW 35 deg to face LEFT spike mark

        lib.dropBottomPixel();

        lib.drive(0, 0, -35, DRIVE_POWER);      // CCW 35 to face forward
        lib.drive(0, -5.5, 0, DRIVE_POWER);       // line up robot to move forward
        lib.drive(30.5, 0, 0, 0.65);       // Drive 35" forward to bridge
        lib.drive(0, 0, -92, 0.65);      // Rotate CCW 90 dg to face back wall
        lib.drive(75, 0, 0, 0.65);      // Drive forward 6' 4" to parking zone, with second pixel
        lib.drive(0, -16, 0, DRIVE_POWER);    // line up to right side
        lib.drive(0, 0, 180, 0.65);           // turn around
        lib.armraisewait(100, 0.2);           // lift arm to backdrop
        lib.reverseFlipper();
        lib.armraisewait(30,0.2);             
        lib.drive(-2.5, 0, 0, DRIVE_POWER);     // drive back to backdrop
        sleep(100);
        lib.openClamp();
        lib.drive(4.5, 0, 0, DRIVE_POWER);    // drive forward to let pixel drop
        lib.normalFlipper();
        lib.armraisewait(-110, 0.4);          // bring arm back down
        lib.armraise(-30,0.2);                
        
        lib.drive(-5, -10, 0, DRIVE_POWER);   // park
      }
      
      telemetry.addLine("Opmode COMPLETE");
      telemetry.update();
      while (opModeIsActive()) {
        sleep(100); // this opMode doesn't use the boilerplate loop pattern, if this changes, be sure to remove the sleep
      }
    }
  }
}
