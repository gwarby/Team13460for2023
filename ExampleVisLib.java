package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ExampleVisLib extends LinearOpMode {
  // private members
  HardwareMap hardwareMap;
  OpenCvWebcam webcam;
  FindPropExamplePipeline findPropPL;

  // pass thru functions for prop detection pipeline
  void FindPropSetEnableDetection(boolean state) { findPropPL.EnableDetection = state; }
  int GetPropXLocation() {return findPropPL.PropXLocation; }
  double FindPropMaxChroma() { return findPropPL.max_chroma; }
  int FindPropMaxX() { return findPropPL.max_x; }
  int FindPropMaxY() { return findPropPL.max_y; }

  // This class isn't an opmode that should be run on the robot
  // but it extends (inherits from) LinearOpMode so it can access
  // hardwareMap & sleep()
  public ExampleVisLib(HardwareMap hwMap, FindPropVisInitData visInit) {
    /************************************************************************
     *   -map hardware specific to our robot
     *   -config hardware settings (like reversed motor directions)
     *   -config camera & OpenCV pipeline used for prop location detection
     ************************************************************************/
    hardwareMap = hwMap;

    int camMonViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), camMonViewId); 

    // specify image processing pipeline to invoke
    // upon receipt of a frame from the camera
    findPropPL = new FindPropExamplePipeline();
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
  
}
