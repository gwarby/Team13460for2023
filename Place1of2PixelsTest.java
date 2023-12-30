package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Place1of2PixelsTest")
public class Place1of2PixelsTest extends LinearOpMode 
{
  double DRIVE_POWER = 0.5;

  private static ElapsedTime programTime = new ElapsedTime();

 /**
   * This function is executed when this OpMode is selected & Init is pressed
   */
  @Override
  public void runOpMode() 
  {
    programTime.reset();
    
    FindPropVisInitData visInitData = new FindPropVisInitData();
    
    AutoCommon lib = new AutoCommon(
      hardwareMap,
      visInitData);

    // Pre Load Pixels:
    lib.preLoadPixels();

    // Look for custom object position while holding 2 pixels:
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

    //double preStopVision1Time = programTime.time();
    //lib.FindPropEnableDetection(false);
    lib.EndPropDetection();  // takes < 70ms experimentally, so we're not going to bother with async
    //double postStopVision1Time = programTime.time();
    //double endPropDetectionTime = postStopVision1Time - preStopVision1Time;
    //telemetry.addData("End Prop Det. Time: ", endPropDetectionTime);
    //telemetry.update();
    //sleep(3000);
    
    
    // TODO: Sanity check that we can still read / access lib.FindPropLocation()
    
    

    // wait for user to press start on Driver Station
    waitForStart();

    if (opModeIsActive()) {
      /************************************************************************
       * START / RUN OPMODE CODE:
       *   -perform autonomous driving based on identified prop location
       ************************************************************************/
      
      if (true) { //(findPropPL.propLocation == "LEFT"){
        /************************************************************************
         *
         * LEFT: (BLUE BACK)
         *
         ************************************************************************/

        // EXTEND ONCE WE ARE IN POSITION & ORIENTATION TO PLACE


        lib.dropBottomPixel();

        lib.armraisewait(100, 0.3);         // raise arm 120 deg (all the way back/up for placing pixel on board)
        lib.reverseFlipper();           // put flipper in rev pos for placing pixel on board
        lib.armraisewait(30, 0.09);        // raise arm last 45 deg
        lib.openClampWait();          // release pixel on board
        sleep(350);                // wait for pixel
        lib.openClampLittleWait();        // open clamp slightly
        lib.groundTransitionFlipper();  // get arm in position to clear axle
        lib.armraisewait(-100, 0.41);        // bring the arm back down
        lib.armraisewait(-20.6, 0.221);      // lower arm back to ground to prevent slamming between programs
                                    // ...leave up ~15 deg from driving to park pos
        lib.armraise(0, 0.159);     // finish lowering claw to ground
        lib.normalFlipper();            // square w/ ground



        // code for putting down arm without raising, 2nd pixel backdrop placement, etc.
        //sleep(800);
        //armraise(0.0,0.2);                                    // the new limiter might prevent getting back to zero
        //sleep(1000);                                          // so wait a beat, then let go of the armraise motor
        //armraise.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // .
        //armraise.setPower(0.0);                               // .
        //sleep(300000);  // end test program



      //} else if (findPropPL.propLocation == "MIDDLE") { // If pixel is in MIDDLE
        /************************************************************************
         *
         * MIDDLE: (BLUE BACK)
         *
         ************************************************************************/
         /*
        normalFlipper();            // square w/ ground
        closeClamp();
        drive(2.5, 0, 0, DRIVE_POWER);        // Fwd 4" to get motors off wall
        groundTransitionFlipper();  // <keep flipper from getting caught>
        armraise(20,0);             // Raise  20 deg for carrying pixels
        sleep(100);
        drive(24.5,0,0,DRIVE_POWER);        // Fwd 15" toward spike marks
        armextend(4,0.47);          // Extend arm 4" reaching pixel over spike
        drive(0,0,-18,DRIVE_POWER);         // CCW 5 deg to be slightly offset from center of MIDDLE spike mark
        dropBottomPixel();              // Drop bottom pixel
        armextend(0, 0.5);            // retract arm
        drive(-4, 0, 0, 0.2);       // back up: don't run over the pixel we just placed,
                                    // ...but also don't go back to far into frames
        drive(0, 0, 108, DRIVE_POWER);      // CW 108 to face away from backdrop
        sleep(50);
        armraise(75, 0.3);         // raise arm 120 deg (all the way back/up for placing pixel on board)
        drive(-24, 0, 0, DRIVE_POWER);       // FWD 25.5" toward backdrop
        reverseFlipper();           // put flipper in rev pos for placing pixel on board
        drive(0, -9, 0, DRIVE_POWER);       // left 10" along backdrop
        armraise(45, 0.127);        // slow down to avoid tipping over
        drive(-2, 0, 0, 0.2);       // REV last 5" to board
        openClampWait();          // release pixel on board
        sleep(400);
        drive(2, 0, 0, DRIVE_POWER); // drive forward 2" to get off wall
        groundTransitionFlipper();  // get arm in position to clear axle
        armraise(-100, 0.41);        // bring the arm back down
        drive(0, 30, 0, DRIVE_POWER); // drive right 18" to wall
        armraise(-20.6, 0.221);      // lower arm further, still leave up ~15 deg for driving
        armextend(0, 0.3);
        drive(-15, 0, 0, DRIVE_POWER); // drive back 5" behind/over line
        armraisewait(0, 0.159);     // finish lowering claw to ground
        normalFlipper();            // square w/ ground
        */

      } else { // RIGHT code
        /************************************************************************
         *
         * RIGHT: (BLUE BACK)
         *
         ************************************************************************/
         /*
        ((DcMotorEx) frontleft).setTargetPositionTolerance(12);
        ((DcMotorEx) rearleft).setTargetPositionTolerance(12);
        ((DcMotorEx) frontright).setTargetPositionTolerance(12);
        ((DcMotorEx) rearright).setTargetPositionTolerance(12);
        ((DcMotorEx) armraise).setTargetPositionTolerance(4);
        ((DcMotorEx) armextend).setTargetPositionTolerance(12);

        normalFlipper();            // square w/ ground
        closeClamp();
        drive(2.5, 0, 0, DRIVE_POWER);        // Fwd 4" to get motors off wall
        groundTransitionFlipper();  // <keep flipper from getting caught>
        armraise(20,0.5);             // Raise  20 deg for carrying pixels
        sleep(100);                    // Give arm time to get off ground
        drive(15.0,0,0,DRIVE_POWER);        // Fwd 15" toward spike marks
        armextend(10.5,0.6);          // Extend arm 4" reaching pixel over spike
        drive(0,0,50,DRIVE_POWER);         // CW 45 deg to face RIGHT spike mark
      // Drop-off
        dropBottomPixel();
        ((DcMotorEx) armraise).setTargetPositionTolerance(12);
        armextend(0,0.31);         // retract arm
        
        // WHAT IS HAPPENING TO POWER??
      //  telemetry.addData(" DRIVE_POWER: ", DRIVE_POWER);
       // telemetry.update();
       // sleep(4000);
        
        drive(-2, 0, 0, DRIVE_POWER); // back up 2" to clear spike mark
        drive(0, 0, 40, DRIVE_POWER);      // CW 40 to face away from backdrop
        sleep(50);
        drive(-24, 0, 0, DRIVE_POWER);       // FWD 26" toward backdrop
        armraise(75, 0.3);         // raise arm 120 deg (all the way back/up for placing pixel on board)
        //drive(0, 0, 180, DRIVE_POWER);      // CW 180 deg to back into backdrop
        reverseFlipper();           // put flipper in rev pos for placing pixel on board
        drive(0, -16.5, 0, DRIVE_POWER);       // left 4" along backdrop
        armraise(45, 0.127);        // slow down to avoid tipping over
        drive(-4, 0, 0, 0.2);       // REV last 5" to board
        openClampLittleWait();          // release pixel on board
        sleep(200);
        drive(2 , 0, 0, 0.2);       // forward 2" from board
        armraise(-100, 0.41);        // bring the arm back down
        drive(0, 36, 0, DRIVE_POWER); // drive right 24" to wall
        groundTransitionFlipperWait();  // get arm in position to clear axle
        armraise(-20.6, 0.221);      // lower arm back to ground to prevent slamming between programs
                                    // ...leave up ~15 deg from driving to park pos
        armextend(0, 0.4);
        drive(-13, 0, 0, DRIVE_POWER); // drive back 5" behind/over line
        openClamp();
        armraisewait(0, 0.159);     // finish lowering claw to ground
        normalFlipperWait();            // square w/ ground
        */
      }

      telemetry.addLine("Opmode COMPLETE");
      telemetry.update();
      while (opModeIsActive()) {
        sleep(100); // this opMode doesn't use the boilerplate loop pattern, if this changes, be sure to remove the sleep
      }
    }
  }
}
