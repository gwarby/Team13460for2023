package org.firstinspires.ftc.teamcode;

public class FindPropVisInitData {
  public int CamStreamWidth_pixels = 544;   // cam width/height probably should not be changed without
  public int CamStreamHeight_pixels = 288;  // corresponding changes the size of the sections & the loop
                                            // boundaries in the FindPropPipeline implementation
  public int ColorChannel = 1;              // channel 1: red, channel 2: blue
  public int MinDeltaDetectionChroma = 25;  // minimum difference between max section chroma & min section chroma to consider a valid object detection
  public int ScanLowestYBlock = 4;          // Y coords: 0 = top of frame, increasing coords go down
                                            // the Prop scan will only happen above this YBlock value
  public int ScanLeftmostXBlock = 4;        // X coords: 0 = left, increasing coords go right
                                            // the Prop scan will only happen to the right of this XBlock value
  public int MidRightXBoundary = 9;          // XBlock line between middle & right prop detections
}