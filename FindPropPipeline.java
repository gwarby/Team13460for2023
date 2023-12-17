package org.firstinspires.ftc.teamcode;

import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


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
public class FindPropPipeline extends OpenCvPipeline
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
