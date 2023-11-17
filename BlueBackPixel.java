package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
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
  private DcMotor frontleft;
  private DcMotor rearleft;
  private DcMotor frontright;
  private DcMotor rearright;
  private Servo grabber;
  DcMotor armextend;
  DcMotor armraise;
  Servo flipper;
  
  OpenCvWebcam webcam;

  double GRABBER_SERVO_OPENED_POS = 0.37;//0.45;
  double GRABBER_SERVO_CLOSED_POS = 0.163;//0.12;
  double OPEN_A_LITTLE = 0.206;//0.22;
  int GRABBER_SERVO_PAUSE_TIME_MS = 355;//450;
  
  double ARM_RAISE_TICKS_PER_DEG = 7.595;
  double ARM_EXTEND_TICKS_PER_INCH = 120.0;
  
  double CLAW_FLIP_SERVO_NORMAL_POS = 0.263;//0.27;
  double CLAW_FLIP_SERVO_TO_FROM_GROUND = 0.24;
  double CLAW_FLIP_SERVO_FLIPPED_POS = 0.93;
  int FLIPPER_SERVO_PAUSE_TIME_MS = 910;//1000;
  
  double DRIVE_POWER = 0.4;

 /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() 
  {
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
        findPropPL.MinDetectionChroma = 132.72;  //142.0; // 142.0 was good with chonky bottom side of prop
        findPropPL.ScanLowestYBlock = 4;
        findPropPL.ScanLeftmostXBlock = 4;
        findPropPL.MidRightXBoundary = 9;
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
        telemetry.addData(" -MAX_CHROMA: ", findPropPL.max_chroma);
        telemetry.addData(" -MAX_X: ", findPropPL.max_x);
        telemetry.addData(" -MAX_Y: ", findPropPL.max_y);
        telemetry.update();
      }
    }

    telemetry.addLine("Waiting for start");
    telemetry.update();

    // wait for user to press start on Driver Station
    waitForStart();

    if (opModeIsActive()) {
      // Put run blocks here.
      
      if (/*findPropPL.propLocation == "LEFT"*/ true){

       normalFlipper(190);            // square w/ ground
      closeClamp();
       sleep(100);
      drive(2.5, 0, 0, DRIVE_POWER);        // Fwd 4" to get motors off wall
      groundTransitionFlipper(85);  // <keep flipper from getting caught>
      armraise(20,0);             // Raise 20 for carrying pixels
      drive(15.0,0,0,DRIVE_POWER);        // Fwd 13.5" toward spike marks
      drive(0,0,-45,DRIVE_POWER);         // CCW -45 deg to face LEFT spike mark
      drive(1, 0, 0, 0.2);        // Fwd 1" - bump a little more toward the spike mark
      armextend(4,0.47);          // Extend arm 4" hopefully reaching pixel over spike
      armraise(-10, 0.15);        // lower arm 10 deg to set pixel stack on spike
                                  // ... slower than default to NOT slam into ground
      //sleep(100);
      normalFlipper(310);            // square w/ ground
      openClampLittle();          // drop pixel stack
      
    // After drop-off
      armraise(0.6, 0);           // Raise 0.6 deg to leave bottom pixel, regrab top
      sleep(100);
      closeClamp();               // Grab the top pixel
      groundTransitionFlipper(220);  // <prevent catching on axle>
      armraise(12, 0);            // raise 12 deg for driving around
      drive(-4, 0, 0, 0.2);       // back up: don't run over the pixel we just placed,
                                  // ...but also don't go back to far into frames
      sleep(100);
      drive(0, 0, -43, DRIVE_POWER);      // CCW -43 to face backdrop
      sleep(100);
      drive(24, 0, 0, DRIVE_POWER);       // FWD 24" toward backdrop
      drive(0, 0, 180, DRIVE_POWER);      // CW 180 deg to back into backdrop
      drive(0, -5, 0, DRIVE_POWER);       // back 4" toward backdrop
      armraise(75, 0.3);         // raise arm 120 deg (all the way back/up for placing pixel on board)
      armraise(45, 0.127);        // slow down to avoid tipping over
      reverseFlipper();           // put flipper in rev pos for placing pixel on board
      armextend(-4,0.31);         // drop pixel from lowest pos to avoid bouncing as much as possible
      drive(-5, 0, 0, 0.2);       // REV last 5" to board
      openClampLittle();          // release pixel on board
      armraise(-100, 0.41);        // bring the arm back down
      groundTransitionFlipper(400);  // get arm in position to clear axle
      armraise(-30.6, 0.221);      // lower arm back to ground to prevent slamming between programs
                                  // ...leave up ~15 deg from driving to park pos
                                  
      // TODO: PARK IN LEFT CORNER, get out of way of alliance partner
                                  
                                  
      openClamp();
      armraise(-10.0, 0.159);     // finish lowering claw to ground
      normalFlipper(190);            // square w/ ground
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
  
  
  
  
  
  // vvvv need this vvvv
  
  
  
  
  
  
  class FindPropPipeline extends OpenCvPipeline
  {
    // Attributes accessed by caller
    int ColorChannel = 2;  // channel 1: red,  channel 2: blue
    String propLocation = "none";
    double max_chroma;
    int max_x = 0, max_y = 0;
    double MinDetectionChroma = 142.0;
    int ScanLowestYBlock = 4;
    int ScanLeftmostXBlock = 4;
    int MidRightXBoundary = 9;
    boolean EnableDetection = false;
    
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
    
    @Override
    public void init(Mat firstFrame)
    {
      // see the SkystoneDeterminationExample.java within
      //    github.com/OpenFTC/EasyOpenCV
      for (int i = 0; i < (SAMPLE_X_SZ-1); i++) {
        for (int j = 0; j < (SAMPLE_Y_SZ-1); j++) {
          // sample overlapping 2x2's
          sampleRects[i][j] = new Rect(new Point(i * X_WIDTH + 0, j * Y_WIDTH + 0), new Point( ((i+1)*(X_WIDTH) + X_WIDTH - 1), ((j+1)*(Y_WIDTH) + Y_WIDTH - 1)));
        }
      }    
    }

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
      for (int i = ScanLeftmostXBlock; i < (SAMPLE_X_SZ-1); ++i) {
        for (int j = 0; j <= ScanLowestYBlock; ++j) { //for (int j = 0; j < (SAMPLE_Y_SZ-1); ++j) {
          if (sampleChromas[i][j] > max_chroma) {
            max_chroma = sampleChromas[i][j];
            max_x = i;
            max_y = j;
          }
        }
      }
      Imgproc.rectangle(ChromaMat, 
        new Point(max_x * X_WIDTH + 0, max_y * Y_WIDTH + 0), new Point( ((max_x+1)*(X_WIDTH) + X_WIDTH - 1), ((max_y+1)*(Y_WIDTH) + Y_WIDTH - 1)),
        new Scalar(240, 199, 185)
        );

      //double MinDetectionChroma = 142.0;
      //int ScanLowestYBlock = 4;
      //int MidRightXBoundary = 9;
      propLocation = "LEFT";
      if (max_chroma > MinDetectionChroma)
      {
        if (max_x > MidRightXBoundary)
          propLocation = "RIGHT";
        else
          propLocation = "MIDDLE";
      }

      /*
      // compare the scores in order to report the position
      if (rightChroma > midChroma && rightChroma > leftChroma)
      {
        propLocation = "RIGHT";
      }
      else if (leftChroma > midChroma && leftChroma > rightChroma)
      {
        propLocation = "LEFT";
      }
      else {
        propLocation = "MIDDLE";
      }
      */
      return ChromaMat;  // return the chroma blue channel w/ rectangles overlaid
    }
  }

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

  private void armraise(double raise_lower, double power) {

    double armRaiseDistance;
    double armRaisePower;
  
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
    //armraise.setPower(0);
  }

  private void armextend(double extend_retract, double power) {

    double armExtendDistance;
    double armExtendPower;
  
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

  //**********************************************
  // GRABBER COMMAND FUNCTIONS
  //**********************************************
  private void closeClamp() { closeClamp(GRABBER_SERVO_PAUSE_TIME_MS); }
  private void closeClamp(int pause_time_ms) {
    grabber.setPosition(GRABBER_SERVO_CLOSED_POS);
    sleep(pause_time_ms);
  }
  private void openClamp() { openClamp(GRABBER_SERVO_PAUSE_TIME_MS); }
  private void openClamp(int pause_time_ms) {
    grabber.setPosition(GRABBER_SERVO_OPENED_POS);
    sleep(pause_time_ms);
  }
  private void openClampLittle() { openClampLittle(GRABBER_SERVO_PAUSE_TIME_MS); }
  private void openClampLittle(int pause_time_ms) {
    grabber.setPosition(OPEN_A_LITTLE);
    sleep(pause_time_ms);
  }
  
  
  //**********************************************
  // FLIPPER COMMAND FUNCTIONS
  //**********************************************
  private void reverseFlipper() { reverseFlipper(FLIPPER_SERVO_PAUSE_TIME_MS); }
  private void reverseFlipper(int pause_time_ms) {
    flipper.setPosition(CLAW_FLIP_SERVO_FLIPPED_POS);
    sleep(pause_time_ms);
  }
  private void normalFlipper() { normalFlipper(FLIPPER_SERVO_PAUSE_TIME_MS); }
  private void normalFlipper(int pause_time_ms) {
    flipper.setPosition(CLAW_FLIP_SERVO_NORMAL_POS);
    sleep(pause_time_ms);
  }
  private void groundTransitionFlipper() { groundTransitionFlipper(FLIPPER_SERVO_PAUSE_TIME_MS); }
  private void groundTransitionFlipper(int pause_time_ms) {
    flipper.setPosition(CLAW_FLIP_SERVO_TO_FROM_GROUND);
    sleep(pause_time_ms);
  }
}
