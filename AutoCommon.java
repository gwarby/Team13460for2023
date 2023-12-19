package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AutoCommon extends LinearOpMode {
  // private members
  HardwareMap hardwareMap;
  OpenCvWebcam webcam;
  FindPropPipeline findPropPL;

  private DcMotor frontleft, rearleft, frontright, rearright, armextend, armraise;
  private Servo grabber, flipper, armlimiter;

  // Constants used to calibrate robot-specific unit conversion, limits, etc.  
  private final double GRABBER_SERVO_OPENED_POS = 0.37;//0.45;
  private final double GRABBER_SERVO_CLOSED_POS = 0.163;//0.12;
  private final double GRABBER_SERVO_OPEN_A_LITTLE_POS = 0.206;//0.22;
  private final int GRABBER_SERVO_PAUSE_TIME_MS = 355;//450;
  
  private final double ARM_RAISE_TICKS_PER_DEG = 7.595;
  private final double ARM_EXTEND_TICKS_PER_INCH = 120.0;
  
  private final double CLAW_FLIP_SERVO_NORMAL_POS = 0.28; //0.263;//0.27;
  private final double CLAW_FLIP_SERVO_TO_FROM_GROUND = 0.24;
  private final double CLAW_FLIP_SERVO_AVOID_ARM_LIMITER = 0.35;
  private final double CLAW_FLIP_SERVO_FLIPPED_POS = 0.93;
  private final int FLIPPER_SERVO_PAUSE_TIME_MS = 910;//1000;
  private final int CALC_GRABBER_WAIT_MS = 1000;
  private final int CALC_FLIPPER_WAIT_MS = 1500;

  private final double ARM_LIMITER_DEACTIVATED = 0.08;
  private final double ARM_LIMITER_ACTIVATED = 0.48;
  
  private final double DRIVE_POWER = 0.5;
  
  // pass thru functions for prop detection pipeline
  void FindPropSetEnableDetection(boolean state) { findPropPL.EnableDetection = state; }
  String FindPropLocation() { return findPropPL.propLocation; }
  double FindPropMaxDeltaChroma() { return findPropPL.max_delta_chroma; }
  double FindPropMaxChroma() { return findPropPL.max_chroma; }
  double FindPropMinChroma() { return findPropPL.min_chroma; }
  int FindPropMaxX() { return findPropPL.max_x; }
  int FindPropMaxY() { return findPropPL.max_y; }

  // This class isn't an opmode that should be run on the robot
  // but it extends (inherits from) LinearOpMode so it can access
  // hardwareMap & sleep()
  public AutoCommon(HardwareMap hwMap, FindPropVisInitData visInit) {
    /************************************************************************
     *   -map hardware specific to our robot
     *   -config hardware settings (like reversed motor directions)
     *   -config camera & OpenCV pipeline used for prop location detection
     ************************************************************************/
    hardwareMap = hwMap;
    frontleft = hardwareMap.get(DcMotor.class, "frontleft");
    rearleft = hardwareMap.get(DcMotor.class, "rearleft");
    frontright = hardwareMap.get(DcMotor.class, "frontright");
    rearright = hardwareMap.get(DcMotor.class, "rearright");
    armraise = hardwareMap.get(DcMotor.class, "armrotate");
    armextend = hardwareMap.get(DcMotor.class, "armextend");
    grabber = hardwareMap.get(Servo.class, "grabber");
    flipper = hardwareMap.get(Servo.class, "flipper");
    armlimiter = hardwareMap.get(Servo.class, "armlimiter");
    
    rearright.setDirection(DcMotorSimple.Direction.REVERSE);
    frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
    armraise.setDirection(DcMotorSimple.Direction.REVERSE);
    armraise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armextend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armextend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    armraise.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    int camMonViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), camMonViewId); 

    // specify image processing pipeline to invoke
    // upon receipt of a frame from the camera
    findPropPL = new FindPropPipeline();
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
        webcam.startStreaming(visInit.CamStreamWidth_pixels, visInit.CamStreamHeight_pixels, OpenCvCameraRotation.UPRIGHT);  //320x240  //432x240   //640x480

        findPropPL.ColorChannel = visInit.ColorChannel;  // channel 1: red,  channel 2: blue
        findPropPL.MinDeltaDetectionChroma = visInit.MinDeltaDetectionChroma;
        findPropPL.ScanLowestYBlock = visInit.ScanLowestYBlock;
        findPropPL.ScanLeftmostXBlock = visInit.ScanLeftmostXBlock;  // originally 4,
                                            // adjust to 1, 2, or 3 for right tile starting positions?
                                            // try not to include left spikemark tape

        findPropPL.MidRightXBoundary = visInit.MidRightXBoundary;   // originally 9,
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
  }
  
  //private void sleep(int ms) { LinearOpMode.sleep(ms); }
  
  // This is a library class, not a proper opMode, but we need access to sleep, so we extend linearOpMode, and as such, must implement runOpMode()
  @Override
  public void runOpMode() { } 
  
    /************************************************************************
    *  PICKUP 2 PIXEL STACK ON INIT - USE ARM LIMITER AS REST
    *     Used to improve readability/ simplify editing loading pixels on init
    *     Uses "arm limiter", which is a lever that rock into place under the
    *     rail of the arm.  This mechanism holds the arm at a very consistent
    *     angle.  This makes dropping 2 pixels & picking back up ONLY the 
    *     top pixel very reliable (especially compared to trusting the 
    *     arm raising motor's encoder modes).
    * 
    *     This has been setup to work with an arm extension of 5".
  *************************************************************************/
  public void preLoadPixels() {
    // ROBOT MOVES ON INIT: (sticker requirement eliminated for CenterStage season)
    deactivateArmLimiter();     // this should be the starting position, but try anyway
    normalFlipper();            // square w/ ground
    sleep(300);
    openClampLittle();          // open clamp slightly
    sleep(400);                 // wait a beat for clamp to open
    closeClamp();               // grasp the pixels
    sleep(400);                 // wait a beat for pixels to be grabbed before lifting arm
    armraisewait(24,0.3);       // Raise above arm limiter
    avoidArmLimiterFlipper();   // rock flipper so angle bracket doesn't rest on arm limiter
    sleep(600);
    activateArmLimiter();       // move arm limiter in place to hold up in "perfect" position
    sleep(550);
    restOnArmLimiter();         // should result in arm ~10 deg?
    normalFlipper();
  }

  /************************************************************************
    *  DROP BOTTOM PIXEL FUNCTION
    *     Used to improve readability/ simplify editing of pixel drop off procedure
  *************************************************************************/
  public void dropBottomPixel() {
    armextendwait(5,0.47);          // Extend arm 4" reaching pixel over spike
    openClampLittleWait();          // drop bottom stack
    sleep(300);
    closeClampWait();               // Grab the top pixel
    sleep(250);
    armextend(0, 0.5);         // retract arm
    armraisewait(16,0.5);             // Raise 16 deg for carrying pixels
    deactivateArmLimiter();
  }

  /************************************************************************
   * COMMON DRIVE FUNCTION:
   *   Used to simplify autonomous programming with simple & readable cmds
   ************************************************************************/
  public void drive(double fwd_bck, double right_left, double cw_ccw, double power) {
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
  public void armraisewait(double raise_lower, double power) {
  
    if (power == 0) {
      power = 0.33;
    }
    if (raise_lower == 0) {
      armraise.setTargetPosition(0);
    } else {
      raise_lower = raise_lower * ARM_RAISE_TICKS_PER_DEG;  // convert 
      raise_lower = (double) ((int)(raise_lower) + armraise.getCurrentPosition());
      armraise.setTargetPosition((int) raise_lower);
    }
    armraise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armraise.setPower(Math.abs(power));
    while (armraise.isBusy()) {
      // Disable telemetry for competition as it slows the loop down
      sleep(10);
    }
    //armraise.setPower(0);  // arm will fall if power is set to 0
  }
  
  public void armraise(double raise_lower, double power) {
  
    if (power == 0) {
      power = 0.33;
    }
    if (raise_lower == 0) {
      armraise.setTargetPosition(0);
    } else {
      raise_lower = raise_lower * ARM_RAISE_TICKS_PER_DEG;  // convert 
      raise_lower = (double) ((int)(raise_lower) + armraise.getCurrentPosition());
      armraise.setTargetPosition((int) raise_lower);
    }
    armraise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armraise.setPower(Math.abs(power));
    //armraise.setPower(0);
  }

  public void restOnArmLimiter() {
    armraise.setTargetPosition(80);  // ?
    armraise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armraise.setPower(0.11);
    sleep(1000);
    armraise.setPower(0.0);
    armraise.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }

  /************************************************************************
   * ARM EXTEND / RETRACT
   * FUNCTIONS:
   ************************************************************************/
  public void armextendwait(double extend_retract, double power) {
    
    if (power == 0) {
      power = 0.33;
    }
    if (extend_retract == 0) {
      armextend.setTargetPosition(0);
    } else {
      extend_retract = extend_retract * ARM_EXTEND_TICKS_PER_INCH;  // convert 
      extend_retract = (double) ((int)(extend_retract) + armextend.getCurrentPosition());
      armextend.setTargetPosition((int) extend_retract);
    }
    armextend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armextend.setPower(Math.abs(power));
    while (armextend.isBusy()) {
      // Disable telemetry for competition as it slows the loop down
      sleep(10);
    }
    //armextend.setPower(0);
  }
  
  public void armextend(double extend_retract, double power) {

    if (power == 0) {
      power = 0.33;
    }
    if (extend_retract == 0) {
      armextend.setTargetPosition(0);
    } else {
      extend_retract = extend_retract * ARM_EXTEND_TICKS_PER_INCH;  // convert 
      extend_retract = (double) ((int)(extend_retract) + armextend.getCurrentPosition());
      armextend.setTargetPosition((int) extend_retract);
    }
    armextend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armextend.setPower(Math.abs(power));
    //armextend.setPower(0);
  }

  /************************************************************************
   * GRABBER COMMAND
   * FUNCTIONS:
   *   1st function: sets servo to run to position, then moves on without wait
   *   2nd function: finds how far the servo is moving, and uses the distance
   *   to calculate a more customized wait time, instead of an arbitrary number
   ************************************************************************/
  public void closeClamp() { grabber.setPosition(GRABBER_SERVO_CLOSED_POS); }
  public void closeClampWait() {
    double distance = Math.abs(grabber.getPosition() - GRABBER_SERVO_CLOSED_POS); // Calculate movement BEFORE setting new position
    grabber.setPosition(GRABBER_SERVO_CLOSED_POS); // Set new position
    sleep((int) (distance * CALC_GRABBER_WAIT_MS + 5)); // Wait distance converted to ms (.3 -> 300 ms), plus 5 for fun. Probably will want to change conversion factor
  }
  
  public void openClamp() { grabber.setPosition(GRABBER_SERVO_OPENED_POS); }
  public void openClampWait() {
    double distance = Math.abs(grabber.getPosition() - GRABBER_SERVO_OPENED_POS);
    grabber.setPosition(GRABBER_SERVO_OPENED_POS);
    sleep((int) (distance * CALC_GRABBER_WAIT_MS + 5)); 
  }
  
  public void openClampLittle() { grabber.setPosition(GRABBER_SERVO_OPEN_A_LITTLE_POS); }
  public void openClampLittleWait() { 
    double distance = Math.abs(grabber.getPosition() - GRABBER_SERVO_OPEN_A_LITTLE_POS);
    grabber.setPosition(GRABBER_SERVO_OPEN_A_LITTLE_POS);
    sleep((int) (distance * CALC_GRABBER_WAIT_MS + 5));
    
  }

  /************************************************************************
   * FLIPPER COMMAND
   * FUNCTIONS:
   *   double function, with & without wait, same as above
   ************************************************************************/
  public void reverseFlipper() { flipper.setPosition(CLAW_FLIP_SERVO_FLIPPED_POS); }
  public void reverseFlipperWait() {
    double distance = Math.abs(flipper.getPosition() - CLAW_FLIP_SERVO_FLIPPED_POS);
    flipper.setPosition(CLAW_FLIP_SERVO_FLIPPED_POS);
    sleep((int) (distance * CALC_FLIPPER_WAIT_MS + 5));
  }
  public void normalFlipper() { flipper.setPosition(CLAW_FLIP_SERVO_NORMAL_POS); }
  public void normalFlipperWait() {
    double distance = Math.abs(flipper.getPosition() - CLAW_FLIP_SERVO_NORMAL_POS);
    flipper.setPosition(CLAW_FLIP_SERVO_NORMAL_POS);
    sleep((int) (distance * CALC_FLIPPER_WAIT_MS + 5));
  }
  public void groundTransitionFlipper() { flipper.setPosition(CLAW_FLIP_SERVO_TO_FROM_GROUND); }
  public void groundTransitionFlipperWait() {
    double distance = Math.abs(flipper.getPosition() - CLAW_FLIP_SERVO_TO_FROM_GROUND);
    flipper.setPosition(CLAW_FLIP_SERVO_TO_FROM_GROUND);
    sleep((int) (distance * CALC_FLIPPER_WAIT_MS + 5));
  }
  public void avoidArmLimiterFlipper() { flipper.setPosition(CLAW_FLIP_SERVO_AVOID_ARM_LIMITER); }
  /************************************************************************
   * ARM LIMITER COMMAND
   * FUNCTIONS:
   *   double function, with & without wait, same as above
   ************************************************************************/
  public void activateArmLimiter() { armlimiter.setPosition(ARM_LIMITER_ACTIVATED); }
  public void deactivateArmLimiter() { armlimiter.setPosition(ARM_LIMITER_DEACTIVATED); }

}
