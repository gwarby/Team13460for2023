package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.openftc.easyopencv.OpenCvPipeline;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "RedFront")
public class RedFront extends LinearOpMode 
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
    
    // The `new IFtcOpmodeSleep()` part of this constructor is giving the AutoCommon lib a way to call `LinearOpMode.sleep()`
    // See AutoCommon.java comments for more information
    AutoCommon lib = new AutoCommon(
      hardwareMap,
      new IFtcOpmodeSleep() { @Override public void FtcSleepFnc(int ms) { sleep(ms); } },
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
         * LEFT: (RED FRONT)
         *
         ************************************************************************/
        lib.drive(17.5, 0, 0,DRIVE_POWER);        // Fwd 15" toward spike marks
        lib.drive(0,0,-45,DRIVE_POWER);         // CCW -45 deg to face LEFT spike mark
        lib.armextendwait(5,0.47);          // Extend arm 4" reaching pixel over spike
        lib.armraisewait(-7.5, 0.2);
        lib.dropBottomPixel();
        lib.armraise(5,0);
        lib.armextend(0, 0.4);          // Retract arm
        lib.drive(0, 6, 0, DRIVE_POWER); // Forward 15" to middle of field
        lib.drive(0, 0, 45, DRIVE_POWER); // Rotate 45 deg back to facing forward
        lib.drive(28, 0, 0, 0.65); // Forward 15" to middle of field
        lib.drive(0, 0, 90, 0.65); // Rotate 90 deg to face back
        lib.drive(78, 0, 0, 0.65); // Forward 5'
        lib.openClamp();
        
        // release motors?

      } else if (lib.FindPropLocation() == "MIDDLE") { // If pixel is in MIDDLE
        /************************************************************************
         *
         * MIDDLE: (RED FRONT)
         *
         ************************************************************************/
        lib.drive(25.0,-5,0,DRIVE_POWER);        // Fwd 15" toward spike marks
        lib.armextendwait(5,0.5);          // Extend arm 4" reaching pixel over spike
        lib.armraisewait(-7.5, 0.2);
        lib.dropBottomPixel();
        lib.armraise(5, 0.1);
        lib.armextend(0, 0.4);          // Retract arm
        lib.drive(0, -10, 0, DRIVE_POWER);
        lib.drive(23, 0, 0, DRIVE_POWER);
        lib.drive(0, 0, 88, DRIVE_POWER);
        lib.drive(94, 0, 0, 0.65);
        lib.openClamp();
        
      } else { // RIGHT code
        /************************************************************************
         *
         * RIGHT: (RED FRONT)
         *
         ************************************************************************/
  
        lib.drive(17.5,0,0,DRIVE_POWER);        // Fwd 15" toward spike 
        lib.drive(0,0,50,DRIVE_POWER);         // CW 45 deg to face RIGHT spike mark
        lib.drive(4, 0, 0, DRIVE_POWER);
        lib.armextendwait(5.0,0.47);          // Extend arm 4" reaching pixel over spike
        lib.armraisewait(-7.5, 0.2);
        lib.dropBottomPixel();
        lib.armraise(5,0);
        lib.armextend(0, 0.4);          // Retract arm
        DRIVE_POWER = 0.65;
        lib.drive(-2, -5, 0, DRIVE_POWER); // Slide left 7" to go around pixel/marker
        lib.drive(0, 0, -50, DRIVE_POWER);
        lib.drive(25, 0, 0, DRIVE_POWER); // Forward 15" to middle of field
        lib.drive(0, 0, 90, DRIVE_POWER); // Rotate 90 deg to face back
        lib.drive(82, 0, 0, DRIVE_POWER); // Forward 5' 7"
        lib.openClamp();
      }

      telemetry.addLine("Opmode COMPLETE");
      telemetry.update();
      while (opModeIsActive()) {
        sleep(100); // this opMode doesn't use the boilerplate loop pattern, if this changes, be sure to remove the sleep
      }
    }
  }
}
