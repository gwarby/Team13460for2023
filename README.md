# Welcome! 
We are Team 13460, the Phantastic Phoenixes, an Iowa-based, First Tech Competition team. In this repository, we have all our actively used code for the 2023/2024 season, CenterStage.
Everything should be thoroughly commented and explained, and we plan to begin releasing official versions around the new year, as we finish up bug fixes and optimization. 
Feel free to use our programs as inspiration or building blocks for your programs. For more information on licensing and copyright, read our Apache 2.0 license.
We wish everyone a great season, and hope we can be of assistance and inspiration to other teams.

## Code naming conventions
Example: redBackPixel
The _red_ refers to this programing being built for the red side. The _Back_ indicates it should be run from the back of the playing field (the side of the playing field with the backdrop/backstage). The _Pixel_ says that the program places a pixel on the backdrop.

*All autonomous programs deliver a pixel to the correct spike mark and park, but only those labeled with _Pixel_ will deliver to the backdrop, all others will deliver to backstage.

## Vision:
We don't use the inbuilt FTC TFOD, we instead use OpenCV. This instead checks for pixels with high red/blue values in the frame, so instead of looking for an object, we're looking for color. We create little overlapping squares on the video feed and check which has the highest red/blue value, to see where our custom element is. Since it's made in one solid color, it shows up strongly. Unfortunately, our camera is only wide enough to see two of the positions accurately, so if nowhere in the frame has a strong enough blue/red value, we assume the element is in the unseen position.

Checkout our branch https://github.com/gwarby/Team13460for2023/tree/NEVER_MERGE/vis_java_block_opencv_example for example vision code that is less specific to our robot & ready to work with blocks.

## Current work:
We are finishing working on IMU and April Tags to account for error and ensure consistent accuracy during autonomous. This is only being implemented in the _front_ programs, in order to be a better teammate, as most teams operate best from the back of the field. For driver-controlled, we are tuning drone launch. This includes setting motor power based on battery outputs (to account for different speeds at different battery voltages), and using April Tags to move to an exact distance from the wall every time, minimizing error. (1/4/24)

## Temporary bugs and fixes:
1. If the Driver Station presenting a blank list of op modes, one time it was fixed entirely with the DS by:
- Use the Android apps tab to shutdown the FTC Driver Station app
- Use the pull down from the top of the Android screen to turn off DS wifi
- Turn back on DS wifi
- Re-open the FTC Driver Station app
