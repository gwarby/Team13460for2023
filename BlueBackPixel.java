package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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


@Autonomous(name = "BlueBackPixel")
public class BlueBackPixel extends LinearOpMode 
{
  // Declare the variables for our hardware, they will be mapped later
  private DcMotor frontleft, rearleft, frontright, rearright, armextend, armraise;
  private Servo grabber, flipper;
  OpenCvWebcam webcam;

  /************************************************************************
   * CALIBRATION CONSTANTS:
   *   Use for common servo positions, motor tick scaling, motor limits,
   *   default times, default powers.
   *   This allows the code to be adjusted only in this section, if
   *   a servo is mounted in a different orientation, for example.
   ************************************************************************/
  double GRABBER_SERVO_OPENED_POS = 0.37;//0.45;
  double GRABBER_SERVO_CLOSED_POS = 0.163;//0.12;
  double GRABBER_SERVO_OPEN_A_LITTLE_POS = 0.206;//0.22;
  int GRABBER_SERVO_PAUSE_TIME_MS = 355;//450;
  
  double ARM_RAISE_TICKS_PER_DEG = 7.595;
  double ARM_EXTEND_TICKS_PER_INCH = 120.0;
  
  double CLAW_FLIP_SERVO_NORMAL_POS = 0.263;//0.27;
  double CLAW_FLIP_SERVO_TO_FROM_GROUND = 0.24;
  double CLAW_FLIP_SERVO_FLIPPED_POS = 0.93;
  int FLIPPER_SERVO_PAUSE_TIME_MS = 910;//1000;
  int CALC_GRABBER_WAIT_MS = 1000;
  int CALC_FLIPPER_WAIT_MS = 1500;
  
  double DRIVE_POWER = 0.5;

 /**
   * This function is executed when this OpMode is selected & Init is pressed
   */
  @Override
  public void runOpMode() 
  {
    /************************************************************************
     * INIT CODE:
     *   -map hardware specific to our robot
     *   -config hardware settings (like reversed motor directions)
     *   -config camera & OpenCV pipeline used for prop location detection
     ************************************************************************/
    frontleft = hardwareMap.get(DcMotor.class, "frontleft");
    rearleft = hardwareMap.get(DcMotor.class, "rearleft");
    frontright = hardwareMap.get(DcMotor.class, "frontright");
    rearright = hardwareMap.get(DcMotor.class, "rearright");
    armraise = hardwareMap.get(DcMotor.class, "armrotate");
    armextend = hardwareMap.get(DcMotor.class, "armextend");
    grabber = hardwareMap.get(Servo.class, "grabber");
    flipper = hardwareMap.get(Servo.class, "flipper");
    
    rearright.setDirection(DcMotorSimple.Direction.REVERSE);
    frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
    armraise.setDirection(DcMotorSimple.Direction.REVERSE);
    armraise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armextend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armextend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    armraise.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    grabber = hardwareMap.get(Servo.class, "grabber");
    
    int camMonViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), camMonViewId); 

    // specify image processing pipeline to invoke
    // upon receipt of a frame from the camera
    FindPropPipeline findPropPL = new FindPropPipeline();
    webcam.setPipeline(findPropPL);

    // open connection to webcam asynchronously
    webcam.setMillisecondsPermissionTimeout(5000);

    // optional: use GPU acceleration
    webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        
    webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
    {
      @Override
      public void onOpened()
      {
        // tell cam to start streaming
        // note: must use resolution supported by cam
        webcam.startStreaming(544, 288, OpenCvCameraRotation.UPRIGHT);  //320x240  //432x240   //640x480

        findPropPL.ColorChannel = 2;  // channel 1: red,  channel 2: blue
        findPropPL.MinDeltaDetectionChroma = 25;
        findPropPL.ScanLowestYBlock = 4;
        findPropPL.ScanLeftmostXBlock = 4;  // originally 4,
                                            // adjust to 1, 2, or 3 for right tile starting positions?
                                            // try not to include left spikemark tape

        findPropPL.MidRightXBoundary = 9;   // originally 9,
                                            // adjust for left/right edge tile starting position once camera is locked in
                                            // try 6, 7 or 8 for right tile starting positions?
                                            // left tile edge starting: BlueBack & RedFront
                                            // right tile edge starting: BlueFront & RedBack
        findPropPL.EnableDetection = true;
      }

      @Override
      public void onError(int errorCode)
      {
        // called if camera can't be opened
      }
    });

    while (!opModeIsActive() & !isStopRequested())
    {
      if (findPropPL.EnableDetection)
      {
        telemetry.addLine("Waiting for start");
        telemetry.addData("PROP LOCATION: ", findPropPL.propLocation);
        telemetry.addData(" -MAX_DELTA_CHROMA: ", findPropPL.max_delta_chroma);
        telemetry.addData(" -MAX_CHROMA: ", findPropPL.max_chroma);
        telemetry.addData(" -MIN_CHROMA: ", findPropPL.min_chroma);
        telemetry.addData(" -MAX_X: ", findPropPL.max_x);
        telemetry.addData(" -MAX_Y: ", findPropPL.max_y);
        telemetry.update();
      }
    }

    findPropPL.EnableDetection = false;

    // wait for user to press start on Driver Station
    waitForStart();

    if (opModeIsActive()) {
      /************************************************************************
       * START / RUN OPMODE CODE:
       *   -perform autonomous driving based on identified prop location
       ************************************************************************/
      
      if (findPropPL.propLocation == "LEFT"){
        /************************************************************************
         *
         * LEFT: (BLUE BACK)
         *
         ************************************************************************/
        normalFlipper();            // square w/ ground
        closeClampWait();
        drive(2.5, 0, 0, DRIVE_POWER);        // Fwd 4" to get motors off wall
        groundTransitionFlipper();  // <keep flipper from getting caught>
        armraisewait(20,0);             // Raise 20 for carrying pixels
        drive(15.0,0,0,DRIVE_POWER);        // Fwd 15" toward spike marks
        drive(0,0,-45,DRIVE_POWER);         // CCW -45 deg to face LEFT spike mark
        drive(1, 0, 0, 0.2);        // Fwd 1" - bump a little more toward the spike mark
        armextend(4,0.47);          // Extend arm 4" hopefully reaching pixel over spike
        normalFlipperWait();            // square w/ ground
        openClampLittleWait();          // drop pixel stack
        // Drop off on spike mark
        armraisewait(0.6, 0);           // Raise 0.6 deg to leave bottom pixel, regrab top
        sleep(50);
        closeClampWait();               // Grab the top pixel
        groundTransitionFlipper();  // <prevent catching on axle>
        armextend(-4, 0.5);         // retract arm
        drive(-4, 0, 0, 0.2);       // back up: don't run over the pixel we just placed,
                                    // ...but also don't go back to far into frames
        drive(0, 0, -43, DRIVE_POWER);      // CCW -43 to face backdrop
        sleep(100);
        armraise(75, 0.3);         // raise arm 120 deg (all the way back/up for placing pixel on board)
        drive(24, 0, 0, DRIVE_POWER);       // FWD 24" toward backdrop
        armraisewait(45, 0.09);        // slow down to avoid tipping over
        drive(0, 0, 180, DRIVE_POWER);      // CW 180 deg to back into backdrop
        reverseFlipper();           // put flipper in rev pos for placing pixel on board
        drive(0, -5, 0, DRIVE_POWER);       // left 5" along backdrop
        drive(-4.3, 0, 0, 0.2);       // REV last 4" to board
        openClampWait();          // release pixel on board
        sleep(350);
        drive(3, 0, 0, DRIVE_POWER); 
        closeClampWait();
        groundTransitionFlipper();  // get arm in position to clear axle
        openClamp();
        armraise(-100, 0.41);        // bring the arm back down
        drive(0, 20, 0, DRIVE_POWER); 
        armraise(-20.6, 0.221);      // lower arm back to ground to prevent slamming between programs
                                    // ...leave up ~15 deg from driving to park pos
        drive(-13, 0, 0, DRIVE_POWER);
        armraise(-10.0, 0.159);     // finish lowering claw to ground
        normalFlipper();            // square w/ ground

      } else if (findPropPL.propLocation == "MIDDLE") { // If pixel is in MIDDLE
        /************************************************************************
         *
         * MIDDLE: (BLUE BACK)
         *
         ************************************************************************/
        normalFlipper();            // square w/ ground
        closeClamp();
        drive(2.5, 0, 0, DRIVE_POWER);        // Fwd 4" to get motors off wall
        groundTransitionFlipper();  // <keep flipper from getting caught>
        armraisewait(20,0);             // Raise 20 for carrying pixels
        drive(24.5,0,0,DRIVE_POWER);        // Fwd 15" toward spike marks
        armextend(4,0.47);          // Extend arm 4" hopefully reaching pixel over spike
        drive(0,0,-18,DRIVE_POWER);         // CCW 5 deg to be slightly offset from center of MIDDLE spike mark
        normalFlipperWait();            // square w/ ground
        openClampLittleWait();          // drop pixel stack
      // After drop-off
        armraisewait(0.6, 0);           // Raise 0.6 deg to leave bottom pixel, regrab top
        sleep(50);
        closeClampWait();               // Grab the top pixel
        armextend(-4, 0.5);
        drive(-4, 0, 0, 0.2);       // back up: don't run over the pixel we just placed,
                                    // ...but also don't go back to far into frames
        drive(0, 0, -72, DRIVE_POWER);      // CCW 83 to face backdrop
        sleep(50);
        armraise(75, 0.3);         // raise arm 120 deg (all the way back/up for placing pixel on board)
        drive(24, 0, 0, DRIVE_POWER);       // FWD 25.5" toward backdrop
        drive(0, 0, 180, DRIVE_POWER);      // CW 180 deg to back into backdrop
        reverseFlipper();           // put flipper in rev pos for placing pixel on board
        drive(0, -9, 0, DRIVE_POWER);       // left 10" along backdrop
        armraisewait(45, 0.127);        // slow down to avoid tipping over
        armextend(-4,0.31);         // drop pixel from lowest pos to avoid bouncing as much as possible
        drive(-2, 0, 0, 0.2);       // REV last 5" to board
        openClampWait();          // release pixel on board
        sleep(400);
        drive(2, 0, 0, DRIVE_POWER); // drive back 5" behind/over line
        groundTransitionFlipper();  // get arm in position to clear axle
        armraisewait(-100, 0.41);        // bring the arm back down
        drive(0, 30, 0, DRIVE_POWER); // drive right 18" to wall
        armraisewait(-20.6, 0.221);      // lower arm back to ground to prevent slamming between programs
                                    // ...leave up ~15 deg from driving to park pos
        drive(-15, 0, 0, DRIVE_POWER); // drive back 5" behind/over line
        armraisewait(-10.0, 0.159);     // finish lowering claw to ground
        normalFlipper();            // square w/ ground

      } else { // RIGHT code
        /************************************************************************
         *
         * RIGHT: (BLUE BACK)
         *
         ************************************************************************/
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
        armraisewait(20,0);             // Raise 20 for carrying pixels
        drive(15.0,0,0,DRIVE_POWER);        // Fwd 15" toward spike marks
        armextend(4.0,0.47);          // Extend arm 4" hopefully reaching pixel over spike
        drive(0,0,50,DRIVE_POWER);         // CW 45 deg to face RIGHT spike mark
        drive(6.5, 0, 0, 0.2);        // Fwd 1" - bump a little more toward the spike mark
      // Drop-off
        normalFlipperWait();            // square w/ ground
        openClampLittleWait();          // drop pixel stack
        sleep(120);
       
      // After drop-off
        armraisewait(0.68, 0);           // Raise 0.6 deg to leave bottom pixel, regrab top
        sleep(50);
        closeClampWait();               // Grab the top pixel
        ((DcMotorEx) armraise).setTargetPositionTolerance(12);
        armextend(-4.0,0.31);         // drop pixel from lowest pos to avoid bouncing as much as possible
        
        // WHAT IS HAPPENING TO POWER??
      //  telemetry.addData(" DRIVE_POWER: ", DRIVE_POWER);
       // telemetry.update();
       // sleep(4000);
        
        drive(-2, 0, 0, DRIVE_POWER); // back up 2" to clear spike mark
        drive(0, 0, -142, DRIVE_POWER);      // CCW 133 to face backdrop
        sleep(50);
        drive(24, 0, 0, DRIVE_POWER);       // FWD 26" toward backdrop
        armraise(75, 0.3);         // raise arm 120 deg (all the way back/up for placing pixel on board)
        drive(0, 0, 180, DRIVE_POWER);      // CW 180 deg to back into backdrop
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
        drive(-13, 0, 0, DRIVE_POWER); // drive back 5" behind/over line
        openClamp();
        armraisewait(-10.0, 0.159);     // finish lowering claw to ground
        normalFlipperWait();            // square w/ ground
      }
      
      while (opModeIsActive()) {
        // Put loop blocks here.

        // do some telemetry
        telemetry.addLine("Opmode COMPLETE");
        telemetry.addData("frames", webcam.getFrameCount());
        telemetry.addData("fps", String.format("%.2f",webcam.getFps()));
        telemetry.addData("theoretical max fps", webcam.getCurrentPipelineMaxFps());
        telemetry.addData("","");
        telemetry.addData("PROP LOCATION: ", findPropPL.propLocation);
        telemetry.addData(" -MAX_CHROMA: ", findPropPL.max_chroma);
        telemetry.addData(" -MAX_X: ", findPropPL.max_x);
        telemetry.addData(" -MAX_Y: ", findPropPL.max_y);
        telemetry.update();

        // don't do this in a real op mode, but for this cam test, chill on the CPU cycles
        sleep(280);
      }
    }
  }

  /************************************************************************
   * VISION PROCESSING / PROP DETECTION PIPELINE:
   *   OpenCV works by attaching 'pipelines' to each streamed camera
   *   frame.
   *   This pipeline currently uses either the red or blue chroma
   *   channel of a YCrCb color format.
   *   This approach was based on:
   *     the SkystoneDeterminationExample.java within
   *     github.com/OpenFTC/EasyOpenCV
   *
   *   Our camera view isn't wide enough to see all 3 prop locations
   *   from both the right tile & left tile starting alignments. So
   *   this pipeline is looking for middle or right props, and
   *   assuming LEFT position otherwise.
   ************************************************************************/
  class FindPropPipeline extends OpenCvPipeline
  {
    /************************************************************************
     * Attributes calibrated by the caller:
     ************************************************************************/
    int ColorChannel = 2;  // channel 1: red,  channel 2: blue
    double MinDeltaDetectionChroma = 12.0;
    int ScanLowestYBlock = 4;
    int ScanLeftmostXBlock = 4;
    int MidRightXBoundary = 9;  // the x block boundary between middle & right
    boolean EnableDetection = false;

    /************************************************************************
     * Attributes read by the caller:
     ************************************************************************/
    String propLocation = "none";  // set to "LEFT", "MIDDLE" or "RIGHT" values
    double max_chroma, min_chroma, max_delta_chroma;
    int max_x = 0, max_y = 0;

    // private attr
    private Mat YCrCb = new Mat();   // Mat for converting color system
    private Mat ChromaMat = new Mat();   // Mat for extracting desired Chroma channel

    // 544, 288   =>  17*32 (aka 17 * 2^5), 9 * 32 (aka 3^2 * 2^5)
    private static final int SAMPLE_X_SZ = 17;
    private static final int SAMPLE_Y_SZ = 9;
    private static final int X_WIDTH = 32;
    private static final int Y_WIDTH = 32;
    private Mat[][] sampleMats = new Mat[SAMPLE_X_SZ][SAMPLE_Y_SZ];
    private Rect[][] sampleRects = new Rect[SAMPLE_X_SZ][SAMPLE_Y_SZ];
    private double[][] sampleChromas = new double[SAMPLE_X_SZ][SAMPLE_Y_SZ];

    /************************************************************************
     * init() creates all the Rect objects at the calculated positions
     ************************************************************************/
    @Override
    public void init(Mat firstFrame)
    {
      for (int i = 0; i < (SAMPLE_X_SZ-1); i++) {
        for (int j = 0; j < (SAMPLE_Y_SZ-1); j++) {
          // sample overlapping 2x2's
          sampleRects[i][j] = new Rect(new Point(i * X_WIDTH + 0, j * Y_WIDTH + 0), new Point( ((i+1)*(X_WIDTH) + X_WIDTH - 1), ((j+1)*(Y_WIDTH) + Y_WIDTH - 1)));
        }
      }    
    }

    /************************************************************************
     * processFrame() scans through each Rect zone looking for max chroma,
     *   min chroma is used to calculate a delta chroma that hopefully
     *   is more consistent than just the raw chroma value if we play
     *   on fields that have different lighting conditions.
     ************************************************************************/
    @Override
    public Mat processFrame(Mat input)
    {
      if (!EnableDetection) {
        return input;
      }
      
      // extracting chroma channel
      Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
      Core.extractChannel(YCrCb, ChromaMat, ColorChannel);  // channel 1: red,  channel 2: blue

      for (int i = 0; i < (SAMPLE_X_SZ-1); ++i) {
        for (int j = 0; j < (SAMPLE_Y_SZ-1); ++j) {
          sampleMats[i][j] = ChromaMat.submat(sampleRects[i][j]);
          sampleChromas[i][j] = Core.mean(sampleMats[i][j]).val[0];
        }
      }

      max_x = 0;
      max_y = 0;
      max_chroma = 0.0; 
      min_chroma = 255.0;
      for (int i = ScanLeftmostXBlock; i < (SAMPLE_X_SZ-1); ++i) {
        for (int j = 0; j <= ScanLowestYBlock; ++j) { //for (int j = 0; j < (SAMPLE_Y_SZ-1); ++j) {
          if (sampleChromas[i][j] > max_chroma) {
            max_chroma = sampleChromas[i][j];
            max_x = i;
            max_y = j;
          }
          if (sampleChromas[i][j] < min_chroma) {
            min_chroma = sampleChromas[i][j];
          }
        }
      }
      max_delta_chroma = max_chroma - min_chroma;
      
      propLocation = "LEFT";
      if (max_delta_chroma > MinDeltaDetectionChroma)
      {
        if (max_x > MidRightXBoundary)
          propLocation = "RIGHT";
        else
          propLocation = "MIDDLE";
      }

      // Draw a rectangle where the prop was detected for manual debugging w/ camera stream
      Imgproc.rectangle(ChromaMat,
              new Point(max_x * X_WIDTH + 0, max_y * Y_WIDTH + 0), new Point( ((max_x+1)*(X_WIDTH) + X_WIDTH - 1), ((max_y+1)*(Y_WIDTH) + Y_WIDTH - 1)),
              new Scalar(240, 199, 185)
      );

      return ChromaMat;  // return the chroma blue channel w/ rectangles overlaid
    }
  }

  /************************************************************************
   * COMMON DRIVE FUNCTION:
   *   Used to simplify autonomous programming with simple & readable cmds
   ************************************************************************/
  private void drive(double fwd_bck, double right_left, double cw_ccw, double power) {
    double frontLeftDistance;
    double rearLeftDistance;
    double frontRightDistance;
    double rearRightDistance;
    double maxDistance;
    double frontLeftPower;
    double frontRightPower;
    double rearLeftPower;
    double rearRightPower;

    frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rearleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rearright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    if (fwd_bck == 0) {
      fwd_bck = 0;
    }
    if (right_left == 0) {
      right_left = 0;
    }
    if (cw_ccw == 0) {
      cw_ccw = 0;
    }
    if (power == 0) {
      power = 0.7;
    }
    fwd_bck = fwd_bck * 54.35714;
    right_left = right_left * 57.04;
    cw_ccw = cw_ccw * 12.85;
    frontLeftDistance = -fwd_bck + right_left -cw_ccw;
    frontRightDistance = -fwd_bck + right_left + cw_ccw;
    rearLeftDistance = -fwd_bck -right_left -cw_ccw;
    rearRightDistance = -fwd_bck - right_left + cw_ccw;
    maxDistance = JavaUtil.maxOfList(JavaUtil.createListWith(frontLeftDistance, frontRightDistance, rearLeftDistance, rearRightDistance));
    frontLeftPower = frontLeftDistance / maxDistance;
    frontRightPower = frontRightDistance / maxDistance;
    rearLeftPower = rearLeftDistance / maxDistance;
    rearRightPower = rearRightDistance / maxDistance;
    frontleft.setTargetPosition((int) frontLeftDistance);
    frontright.setTargetPosition((int) frontRightDistance);
    rearleft.setTargetPosition((int) rearLeftDistance);
    rearright.setTargetPosition((int) rearRightDistance);
    frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rearleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rearright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontleft.setPower(Math.abs(frontLeftPower * power));
    frontright.setPower(Math.abs(frontRightPower * power));
    rearleft.setPower(Math.abs(rearLeftPower * power));
    rearright.setPower(Math.abs(rearRightPower * power));
    while (frontleft.isBusy() || frontright.isBusy() || rearleft.isBusy() || rearright.isBusy()) {
      // Disable telemetry for competition as it slows the loop down
      if (true) {
        sleep(10);
      }
    }
    frontleft.setPower(0);
    frontright.setPower(0);
    rearleft.setPower(0);
    rearright.setPower(0);
  }

  /************************************************************************
   * ARM RAISE / LOWER
   * FUNCTIONS:
   ************************************************************************/
  private void armraisewait(double raise_lower, double power) {
    if (raise_lower == 0) {
      raise_lower = 0;
    }
    if (power == 0) {
      power = 0.33;
    }
    raise_lower = raise_lower * ARM_RAISE_TICKS_PER_DEG;  // convert 
    raise_lower = (double) ((int)(raise_lower) + armraise.getCurrentPosition());
    armraise.setTargetPosition((int) raise_lower);
    armraise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armraise.setPower(Math.abs(power));
    while (armraise.isBusy()) {
      // Disable telemetry for competition as it slows the loop down
      sleep(10);
    }
    //armraise.setPower(0);  // arm will fall if power is set to 0
  }

  private void armraise(double raise_lower, double power) {
    if (raise_lower == 0) {
      raise_lower = 0;
    }
    if (power == 0) {
      power = 0.33;
    }
    raise_lower = raise_lower * ARM_RAISE_TICKS_PER_DEG;  // convert 
    raise_lower = (double) ((int)(raise_lower) + armraise.getCurrentPosition());
    armraise.setTargetPosition((int) raise_lower);
    armraise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armraise.setPower(Math.abs(power));
  }

  /************************************************************************
   * ARM EXTEND / RETRACT
   * FUNCTIONS:
   ************************************************************************/
  private void armextendwait(double extend_retract, double power) {

    if (extend_retract == 0) {
      extend_retract = 0;
    }
    if (power == 0) {
      power = 0.33;
    }
    extend_retract = extend_retract * ARM_EXTEND_TICKS_PER_INCH;  // convert
    extend_retract = (double) ((int)(extend_retract) + armextend.getCurrentPosition());
    armextend.setTargetPosition((int) extend_retract);
    armextend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armextend.setPower(Math.abs(power));
    while (armextend.isBusy()) {
      // Disable telemetry for competition as it slows the loop down
      sleep(10);
    }
    //armextend.setPower(0);
  }

  private void armextend(double extend_retract, double power) {
    if (extend_retract == 0) {
      extend_retract = 0;
    }
    if (power == 0) {
      power = 0.33;
    }
    extend_retract = extend_retract * ARM_EXTEND_TICKS_PER_INCH;  // convert 
    extend_retract = (double) ((int)(extend_retract) + armextend.getCurrentPosition());
    armextend.setTargetPosition((int) extend_retract);
    armextend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armextend.setPower(Math.abs(power));
    while (armextend.isBusy()) {
      // Disable telemetry for competition as it slows the loop down
      sleep(10);
    }
    //armextend.setPower(0);
  }

  /************************************************************************
   * GRABBER COMMAND
   * FUNCTIONS:
   *   1st function: sets servo to run to position, then moves on without wait
   *   2nd function: finds how far the servo is moving, and uses the distance
   *   to calculate a more customized wait time, instead of an arbitrary number
   ************************************************************************/
  private void closeClamp() { grabber.setPosition(GRABBER_SERVO_CLOSED_POS); }
  private void closeClampWait() {
    double distance = Math.abs(grabber.getPosition() - GRABBER_SERVO_CLOSED_POS); // Calculate movement BEFORE setting new position
    grabber.setPosition(GRABBER_SERVO_CLOSED_POS); // Set new position
    sleep((int) (distance * CALC_GRABBER_WAIT_MS + 5)); // Wait distance converted to ms (.3 -> 300 ms), plus 5 for fun. Probably will want to change conversion factor
  }
  
  private void openClamp() { grabber.setPosition(GRABBER_SERVO_OPENED_POS); }
  private void openClampWait() {
    double distance = Math.abs(grabber.getPosition() - GRABBER_SERVO_OPENED_POS);
    grabber.setPosition(GRABBER_SERVO_OPENED_POS);
    sleep((int) (distance * CALC_GRABBER_WAIT_MS + 5)); 
  }
  
  private void openClampLittle() { grabber.setPosition(GRABBER_SERVO_OPEN_A_LITTLE_POS); }
  private void openClampLittleWait() { 
    double distance = Math.abs(grabber.getPosition() - GRABBER_SERVO_OPEN_A_LITTLE_POS);
    grabber.setPosition(GRABBER_SERVO_OPEN_A_LITTLE_POS);
    sleep((int) (distance * CALC_GRABBER_WAIT_MS + 5));
    
  }

  /************************************************************************
   * FLIPPER COMMAND
   * FUNCTIONS:
   *   double function, with & without wait, same as above
   ************************************************************************/
  private void reverseFlipper() { flipper.setPosition(CLAW_FLIP_SERVO_FLIPPED_POS); }
  private void reverseFlipperWait() {
    double distance = Math.abs(flipper.getPosition() - CLAW_FLIP_SERVO_FLIPPED_POS);
    flipper.setPosition(CLAW_FLIP_SERVO_FLIPPED_POS);
    sleep((int) (distance * CALC_FLIPPER_WAIT_MS + 5));
  }
  private void normalFlipper() { flipper.setPosition(CLAW_FLIP_SERVO_NORMAL_POS); }
  private void normalFlipperWait() {
    double distance = Math.abs(flipper.getPosition() - CLAW_FLIP_SERVO_NORMAL_POS);
    flipper.setPosition(CLAW_FLIP_SERVO_NORMAL_POS);
    sleep((int) (distance * CALC_FLIPPER_WAIT_MS + 5));
  }
  private void groundTransitionFlipper() { flipper.setPosition(CLAW_FLIP_SERVO_TO_FROM_GROUND); }
  private void groundTransitionFlipperWait() {
    double distance = Math.abs(flipper.getPosition() - CLAW_FLIP_SERVO_TO_FROM_GROUND);
    flipper.setPosition(CLAW_FLIP_SERVO_TO_FROM_GROUND);
    sleep((int) (distance * CALC_FLIPPER_WAIT_MS + 5));
  }
}
