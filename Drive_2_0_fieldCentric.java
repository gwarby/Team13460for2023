package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// @Disabled

@TeleOp(name = "Drive_2_0_fieldCentric")
public class Drive_2_0_fieldCentric extends LinearOpMode {

  // CONSTANTS USED TO ADJUST PROGRAM:
  double MAX_DRIVE_MOTOR_POWER = 0.75;  // up to 1.0
  boolean IS_FIELD_CENTRIC = false;
  double MAX_ARM_RAISE_POWER = 0.45;
  double MAX_ARM_LOWER_POWER = 0.16;
  int MAX_ARM_RAISE_TICKS = 1200;
  int MIN_ARM_RAISE_TICKS_WHEN_NOT_PICKING_UP = 150;
  int MIN_ARM_RAISE_TICKS = 20;
  double STICK_DEADZONE = 0.011;
  double GRAB_TOP_PIXEL_TICKS = 4;

  double MAX_ARM_EXTEND_POWER = 0.45;
  double MAX_ARM_RETRACT_POWER = 0.45;
  int MAX_ARM_EXTEND_TICKS = 2100;
  int MIN_ARM_EXTEND_TICKS_WHEN_NOT_PICKING_UP = 120;
  
  int FLIP_TIME_DEBOUNCE_MS = 350;
  double CLAW_FLIP_SERVO_NORMAL_POS = 0.27;
  double CLAW_FLIP_SERVO_FLIPPED_POS = 0.93;
  
  int GRABBER_TIME_DEBOUNCE_MS = 350;
  double GRABBER_SERVO_OPENED_POS = 0.35;
  double GRABBER_SERVO_CLOSED_POS = 0.16;
  double GRABBER_SERVO_OPENED_A_LITTLE_POS = .206;
  double GRABBER_BOTTOM_PIXEL_DEBOUNCE_MS = 350;

  double DRONE_SERVO_LOAD_POS = 0.6;
  double DRONE_SERVO_LAUNCH_POS = 0.3;

  //   An ElapsedTime'r for operations that should wait without pausing the loop
  private ElapsedTime currentTime = new ElapsedTime();
  

  /**`
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    // INIT:
    
    // :find our HW in the hardwareMap:
    //   IMU:
    IMU imu = hardwareMap.get(IMU.class, "imu");  // Retrieve the IMU from the hardware map
    imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
    //   Drive Motors:
    DcMotor frontright = hardwareMap.get(DcMotor.class, "frontright");
    DcMotor rearleft = hardwareMap.get(DcMotor.class, "rearleft");
    DcMotor frontleft = hardwareMap.get(DcMotor.class, "frontleft");
    DcMotor rearright = hardwareMap.get(DcMotor.class, "rearright");
    //   Arm extend & arm rotate up/down motors:
    DcMotor armextend = hardwareMap.get(DcMotor.class, "armextend");
    DcMotor armraise = hardwareMap.get(DcMotor.class, "armrotate");
    //   Servos for grabbing & flipping (i.e. rotating) the grabber:
    Servo flipper = hardwareMap.get(Servo.class, "flipper");
    Servo grabber = hardwareMap.get(Servo.class, "grabber");
    //   Motor for end game lifting of the robot
    DcMotor lifter = hardwareMap.get(DcMotor.class, "lifter");
    // Servo for launching drone
    Servo droneLauncher = hardwareMap.get(Servo.class, "dronelauncher");
    
    // :set motor directions so that pos/neg tick encoder positions make sense
    frontright.setDirection(DcMotorSimple.Direction.REVERSE);
    rearleft.setDirection(DcMotorSimple.Direction.REVERSE);
    lifter.setDirection(DcMotorSimple.Direction.REVERSE);
    armraise.setDirection(DcMotorSimple.Direction.REVERSE);
    // :reset motor encoder positions to zero
    armraise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armextend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // :set initial motor run modes, if not set in the loop
    armextend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    armraise.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    // :config any additional motor parameters
    armextend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    armraise.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    // Variables for driver commands
    double driverCmd_Fwd, driverCmd_Right, driverCmd_Rotate;
    double driverCmd_ArmRaise, driverCmd_ArmExtend;
    boolean driverCmd_GrabberToggle, driverCmd_ClawFlipToggle;
    double driverCmd_RaiseLifterHooks, driverCmd_LowerLifterHooks;
    boolean driverCmd_AutoHoldLifterHooks;
    boolean driverCmd_ArmToGround, driverCmd_GrabTopPixel, driverCmd_ArmToHolding, driverCmd_DropBottomPixel;
    boolean driverCmd_LaunchDrone, driverCmd_LoadDrone;
    // Variables for current positions, headings, etc.
    int armExtendPositionTicks;
    int armRaisePositionTicks;
    
    // Variables for control targets - positions, speeds, headings, etc.
    double robotCmd_Fwd, robotCmd_Right, robotCmd_Rotate;
    boolean isArmHolding = false;  // needs to maintain state, so must be outside loop
    int armRaiseTargetPosition = 0;
    boolean isArmExtendHolding = false;
    int armExtendTargetPosition = 0;

    boolean isClawInFlippedPosition = false; // false is normal position
    int lastTimeFlippedMs = 0; // used to unbounce flipper command

    boolean isGrabberInClosedPosition = true; // true is closed
    int lastTimeGrabberToggledMs = 0;
    int lastTimeBottomPixelMs = 0;
    
    boolean lifterUp = false; // false is down
    int lastTimeLifted = 0; // used to unbounce lifter command
    
    int loopsExecuted = 0;
    double prevTimeMs = 0.0;
    //ElapsedTime currentTime = new ElaspedTime();
    // Set flipper to normal position
    flipper.setPosition(CLAW_FLIP_SERVO_NORMAL_POS);
    // Pull droneLauncher servo back to load drone
    droneLauncher.setPosition(DRONE_SERVO_LOAD_POS);
    waitForStart();
    // AFTER START, BEFORE LOOP:
    // TODO: minor arm move: raise, extend, lower but keep slightly off ground
    while (opModeIsActive()) {  // START OF LOOP
      double currTimeMs = currentTime.milliseconds();
      // DRIVE CONTROLS MAP
      // :mechanum drive 
      driverCmd_Right=gamepad1.right_stick_x;
      driverCmd_Fwd=-gamepad1.right_stick_y;  // negative because stick_y is up=neg, we want up=pos
      driverCmd_Rotate=gamepad1.left_stick_x;
      telemetry.addData("driverCmdFwd", driverCmd_Fwd);
      // :arm rotate (aka raise/lower) & arm extend
      driverCmd_ArmRaise = -gamepad2.left_stick_y;
      driverCmd_ArmExtend = -gamepad2.right_stick_y;
      driverCmd_ArmToGround = gamepad2.dpad_down;
      driverCmd_GrabTopPixel = gamepad2.dpad_left || gamepad2.dpad_right;
      driverCmd_ArmToHolding = gamepad2.dpad_up;
      // :claw grab & flip 
      driverCmd_GrabberToggle = gamepad2.a;
      driverCmd_ClawFlipToggle = gamepad2.right_bumper;
      // :lifter aka hanger
      driverCmd_RaiseLifterHooks = gamepad2.right_trigger;
      driverCmd_LowerLifterHooks = gamepad2.left_trigger;
      driverCmd_AutoHoldLifterHooks = gamepad2.dpad_down;
      driverCmd_DropBottomPixel = gamepad2.left_bumper;
      
      driverCmd_LaunchDrone = gamepad1.y && gamepad2.y;
      driverCmd_LoadDrone = gamepad1.x && gamepad2.x;


      // CONTROL CODE

      // :mechanum drive
      double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);  // Get the bot heading from the IMU
      telemetry.addData("Robot Heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
      telemetry.addData("Robot Roll", imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
      telemetry.addData("Robot Pitch", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));

      if (IS_FIELD_CENTRIC) {
        // Rotate the movement direction counter to the bot's rotation
        robotCmd_Right = driverCmd_Right * Math.cos(-botHeading) - driverCmd_Fwd * Math.sin(-botHeading);
        robotCmd_Fwd = driverCmd_Right * Math.sin(-botHeading) + driverCmd_Fwd * Math.cos(-botHeading);
        robotCmd_Rotate = driverCmd_Rotate;
      }
      else {  // normal, robot centric:
        robotCmd_Right = driverCmd_Right;
        robotCmd_Fwd = driverCmd_Fwd;
        robotCmd_Rotate = driverCmd_Rotate;
      }
      // Denominator is the largest motor power (absolute value) or MAX_DRIVE_MOTOR_POWER
      // This ensures all the powers maintain the same ratio,
      // but only if at least one is out of the range [-MAX_DRIVE_MOTOR_POWER, MAX_DRIVE_MOTOR_POWER]
      double denominator = Math.max(Math.abs(robotCmd_Fwd) + Math.abs(robotCmd_Right) + Math.abs(robotCmd_Rotate), 1.0);
      double frontLeftPower = MAX_DRIVE_MOTOR_POWER * (robotCmd_Fwd - robotCmd_Right + robotCmd_Rotate) / denominator;
      double frontRightPower = MAX_DRIVE_MOTOR_POWER * (robotCmd_Fwd - robotCmd_Right - robotCmd_Rotate) / denominator;
      double backLeftPower = MAX_DRIVE_MOTOR_POWER * (robotCmd_Fwd + robotCmd_Right + robotCmd_Rotate) / denominator;
      double backRightPower = MAX_DRIVE_MOTOR_POWER * (robotCmd_Fwd + robotCmd_Right - robotCmd_Rotate) / denominator;
      // Set motor powers
      frontleft.setPower(frontLeftPower);
      frontright.setPower(frontRightPower);
      rearleft.setPower(backLeftPower);
      rearright.setPower(backRightPower);
      


      // Raise/Lower Arm (rotate motor) code
      armRaisePositionTicks = armraise.getCurrentPosition(); // set to current position of rotate motor
      telemetry.addData("Arm Raise Pos:", armRaisePositionTicks);

      boolean isArmCmdNone = Math.abs(driverCmd_ArmRaise) <= STICK_DEADZONE;
      boolean isArmCmdUp = driverCmd_ArmRaise > STICK_DEADZONE;
      boolean isArmCmdDown = driverCmd_ArmRaise < -STICK_DEADZONE;

      boolean isArmTooLow = armRaisePositionTicks < MIN_ARM_RAISE_TICKS;
      boolean isArmTooHigh = armRaisePositionTicks > MAX_ARM_RAISE_TICKS;
      boolean isArmNearLowLimit = armRaisePositionTicks < (MIN_ARM_RAISE_TICKS_WHEN_NOT_PICKING_UP);
      boolean isArmNearHighLimit = armRaisePositionTicks > (MAX_ARM_RAISE_TICKS - 25);
      
      // boolean isArmHolding = false;  // needs to maintain state, so must be outside loop
      if (isArmCmdNone) {  // hold within limits
        if (!isArmHolding) {  // entering hold state
          //   without this block, we keep updating the
          //   target position = to the current position,
          //   causing the hold to drift due to gravity & bouncing
          isArmHolding = true;

          armRaiseTargetPosition = armRaisePositionTicks;
          if (isArmTooLow) {
            armRaiseTargetPosition = MIN_ARM_RAISE_TICKS_WHEN_NOT_PICKING_UP;
          } else if (isArmTooHigh) {
            armRaiseTargetPosition = MAX_ARM_RAISE_TICKS;
          }
          armraise.setTargetPosition(armRaiseTargetPosition);
          armraise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          armraise.setPower(0.265);
        } else {
          armraise.setTargetPosition(armRaiseTargetPosition);
          armraise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          armraise.setPower(0.265);
        }
      } else if (isArmCmdUp) {
        isArmHolding = false;
        if (isArmTooHigh || isArmNearHighLimit) {
          armraise.setTargetPosition(MAX_ARM_RAISE_TICKS);
          armraise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          armraise.setPower(0.265);
        } else {
          armraise.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          armraise.setPower(driverCmd_ArmRaise * MAX_ARM_RAISE_POWER);
        }
      } else {  // isArmCmdDown
        isArmHolding = false;
        if (isArmTooLow || isArmNearLowLimit) {
        armraise.setTargetPosition(MIN_ARM_RAISE_TICKS_WHEN_NOT_PICKING_UP);
        armraise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armraise.setPower(0.265);
        } else {
          armraise.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          armraise.setPower(driverCmd_ArmRaise * MAX_ARM_LOWER_POWER);
        }
      }
      
      // run arm to set positions using buttons
      // a -> ground, b -> second stacked pixel, y -> holding
      // x grabs/ lets go of pixel (still)
      if (driverCmd_ArmToGround) { // send the arm all the way to the ground
        isArmHolding = true;
        armRaiseTargetPosition = 0;
      } else if (driverCmd_GrabTopPixel) { // grab top pixel in stack of two
        isArmHolding = true;
        armRaiseTargetPosition = (int) GRAB_TOP_PIXEL_TICKS;
      } else if (driverCmd_ArmToHolding) { // Put arm back to holding position
        isArmHolding = true;
        armRaiseTargetPosition = MIN_ARM_RAISE_TICKS_WHEN_NOT_PICKING_UP;
      }
      

      // Extend Arm code
      armExtendPositionTicks = armextend.getCurrentPosition(); // set to current position of rotate motor
      telemetry.addData("Arm Extend Pos:", armExtendPositionTicks);

      boolean isArmCmdExtendNone = Math.abs(driverCmd_ArmExtend) <= STICK_DEADZONE;
      boolean isArmCmdExtend = driverCmd_ArmExtend > STICK_DEADZONE;
      boolean isArmCmdRetract = driverCmd_ArmExtend < -STICK_DEADZONE;

      boolean isArmTooRetracted = armExtendPositionTicks < MIN_ARM_EXTEND_TICKS_WHEN_NOT_PICKING_UP;
      boolean isArmTooExtended = armExtendPositionTicks > MAX_ARM_EXTEND_TICKS;
      boolean isArmNearRetractLimit = armExtendPositionTicks < (MIN_ARM_EXTEND_TICKS_WHEN_NOT_PICKING_UP + 25);
      boolean isArmNearExtendLimit = armExtendPositionTicks > (MAX_ARM_EXTEND_TICKS - 25);
      
      // boolean isArmHolding = false;  // needs to maintain state, so must be outside loop

      if (isArmCmdExtendNone) {  // hold within limits
        telemetry.addData("extend:","holding");
        if (!isArmExtendHolding) {  // entering hold state
          //   without this block, we keep updating the
          //   target position = to the current position,
          //   causing the hold to drift due to gravity & bouncing
          isArmExtendHolding = true;

          armExtendTargetPosition = armExtendPositionTicks;
          if (isArmTooRetracted) {
            armExtendTargetPosition = MIN_ARM_EXTEND_TICKS_WHEN_NOT_PICKING_UP;
          }
          else if (isArmTooExtended) {
            armExtendTargetPosition = MAX_ARM_EXTEND_TICKS;
          }
        }
        else {
          armextend.setTargetPosition(armExtendTargetPosition);
          armextend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          armextend.setPower(0.265);
        }
      }
      else if (isArmCmdExtend) {
        telemetry.addData("extend:","extending");
        isArmExtendHolding = false;
        if (isArmTooExtended || isArmNearExtendLimit) {
          armextend.setTargetPosition(MAX_ARM_EXTEND_TICKS);
          armextend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          armextend.setPower(0.265);
        }
        else {
          armextend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          armextend.setPower(driverCmd_ArmExtend * MAX_ARM_EXTEND_POWER);
        }
      }
      else {  // isArmCmdRetract
        telemetry.addData("extend:","retracting");
        isArmExtendHolding = false;
        if (isArmTooRetracted || isArmNearRetractLimit) {
          armextend.setTargetPosition(MIN_ARM_EXTEND_TICKS_WHEN_NOT_PICKING_UP);
          armextend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          armextend.setPower(0.265);
        }
        else {
          armextend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          armextend.setPower(driverCmd_ArmExtend * MAX_ARM_RETRACT_POWER);
        }
      }

      
      // Claw Flip / Rotater:
      boolean isFlipDebounceTimeElapsed = ((int) currentTime.milliseconds() - lastTimeFlippedMs) 
                                          > FLIP_TIME_DEBOUNCE_MS;

      if (driverCmd_ClawFlipToggle && isFlipDebounceTimeElapsed) {
        if (isClawInFlippedPosition) { // set to default position
          isClawInFlippedPosition = false;
          flipper.setPosition(CLAW_FLIP_SERVO_NORMAL_POS);
        } 
        else { // rotate to 180 degrees
          isClawInFlippedPosition = true;
          flipper.setPosition(CLAW_FLIP_SERVO_FLIPPED_POS);
        }
        lastTimeFlippedMs = (int) currentTime.milliseconds();  // refresh 'lastTime', regardless of which direction we went
      }

      // grabber servo code
      boolean isGrabberDebounceTimeElapse =  ((int) currentTime.milliseconds() - lastTimeGrabberToggledMs)
                                             > GRABBER_TIME_DEBOUNCE_MS;
      
      if (driverCmd_GrabberToggle && isGrabberDebounceTimeElapse)
      {
        if (isGrabberInClosedPosition){
          isGrabberInClosedPosition = false;
          grabber.setPosition(GRABBER_SERVO_OPENED_POS);  // open
        } else {
          isGrabberInClosedPosition = true;
          grabber.setPosition(GRABBER_SERVO_CLOSED_POS);
        }
        lastTimeGrabberToggledMs = (int) currentTime.milliseconds();  // refresh 'lastTime', regardless of which direction we went
      }

      // drop bottom pixel code
      boolean isBottomPixelDebounceTimeElapse = ((int) currentTime.milliseconds() - lastTimeBottomPixelMs)
                                                                                > GRABBER_BOTTOM_PIXEL_DEBOUNCE_MS;

      if (driverCmd_DropBottomPixel ) {
        flipper.setPosition(CLAW_FLIP_SERVO_NORMAL_POS);
        grabber.setPosition(GRABBER_SERVO_OPEN_A_LITTE_POS);
        sleep(100);
        grabber.setPosition(GRABBER_SERVO_OPENED_POS);
        lastTimeBottomPIxelMs = currentTime.milliseconds();
      }
      
      // LIFTER (aka robot HANGING)
      lifter.setPower(driverCmd_RaiseLifterHooks - driverCmd_LowerLifterHooks);
      telemetry.addData("lifter pos:", lifter.getCurrentPosition());

      //if (gamepad2.dpad_down) { // reset lifter to 0 (not always necessary)
      //  lifter.setTargetPosition(0);
      //  lifter.setPower(1);
      //  lifterUp = false;
      //}

      // Drone launch control: both driver 1 and 2 must push Dpad up, and must be 90 sec into match
      if (driverCmd_LaunchDrone && (currTimeMs/1000) > 90 ) {
        droneLauncher.setPosition(DRONE_SERVO_LAUNCH_POS);
      }
      
      if (driverCmd_LoadDrone) {
        droneLauncher.setPosition(DRONE_SERVO_LOAD_POS);
      }

      loopsExecuted += 1;
      telemetry.addData("loops:", loopsExecuted);
      telemetry.addData("time delta (ms):", currTimeMs - prevTimeMs);
      telemetry.addData("time:", currTimeMs / 1000.0);
      prevTimeMs = currTimeMs;
      telemetry.update();
    }  // END OF WHILE LOOP
  }  // END OF RUN OPMODE FUNCTION
}  // END OF CLASS DEF

