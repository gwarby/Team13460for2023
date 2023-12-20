# Welcome! 
We are Team 13460, the Phantastic Phoenixes, an Iowa-based, First Tech Competition team. 

## OpenCV Vision Example:
In this repository branch, we have example code using OpenCV to detect "the most blue" or "the most red" section of the webcam feed.  The highest level java functions have decorations that make them available as Blocks for teams that aren't using java.

## Usage:
 - Go to the 'OnBotJava' tab and use the Upload button to upload all the .java files
 - It is likely that you will need to also upload the 2 files from our AuxJavaFiles folder.  See notes in next section.
 - There is a red wrench icon toward the lower left of the of OnBotJava screens that will try to build all of the java code on the robot - click it
 - If the build succeeds & you are using Java:
   - We recommend checking out BlueRedObjectDetectBlocks.java first, as it is the highest level of functions
 - If the build succeeds & you are using Blocks:
   - Go back to the 'Blocks' tab, open a Blocks OpMode & look for the 'Java Classes' section in the Block libraries
 - If the build fails:
   - You can always delete these .java files from the file list in the 'OnBotJava' tab (expanding the folders/sections can be a little tricky)
   - You can try having your team's coaches or mentors reach out to team mentor @lcdnbl (but the FTC forums might get a more timely response!)

## Extra OpenCV Classes:
These extra java files provide some OpenCV classes that our robot was missing (while running App Version 9.0.1).  We have grabbed copies of these files from the OpenCV source.  This seems likely to be fixed in future.  The versions we have here appear to already be outdated compared to https://github.com/OpenFTC/EasyOpenCV, so please use your own discretion when picking up our copies.  You might be setting yourself up for a cryptic error at that start of next season or something similar.

## What's in our Java Files:
 - BlueRedObjectDetectBlocks.java:
   - top level functions including:
     - InitPropDetection: starts vision pipeline & starts looking for most blue or most red section
       - argument: colorCh : 1=red, 2=blue
     - GetPropXLocation(): returns the result of the last streamed frame processing
       - return value: approximate x pixel location of the center of the rectangle that was detected, because the overlapping squares are 64 pixels wide, these will be multiples of 32
     - DisablePropDetection(): causes this OpenCV pipeline to stop processing frames, hopefully cutting down cpu usage
 - ExampleVisLib.java
   - Mostly init code that finds the webcam (which is hardcoded to have the name "Webcam" on line 36), creates our example pipeline, and connects them both.  This code is complicated with 'lambda functions' and objects that implement the Async/Listener pattern, but it is pretty directly copied from the OpenCV examples (don't let the code in this file deter you from trying java).
   - Lines 18 to 22 hint at some additional data the pipeline reports that you might want to use
 - FindPropExamplePipeline.java
   - Our OpenCV pipeline.  We highly recommend reading the OpenCV docs to understand the library functions of OpenCv that we have used.  We hope our comments are helpful, if you choose to examine how we're processing each webcam frame.
 - FindPropVisInitData.java:
   - A simple data class with some default configuration values.  Many of the properties are leftover from our bot-specific competition code & aren't used here.

## Background on Vision Library Choice:
We don't use the inbuilt FTC TFOD, we instead use OpenCV. This checks for pixels with high red/blue values in the frame, so instead of looking for a specific object, we're looking for color. We create overlapping squares on the video feed and check which has the highest mean red/blue value, to see where our custom element is. Since it's made in one solid color, it shows up strongly. 
