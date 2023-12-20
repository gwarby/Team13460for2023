package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;

public class BlueRedObjectDetectBlocks extends BlocksOpModeCompanion {
    static ExampleVisLib visLib = null;

    @ExportToBlocks (
        parameterLabels = {"colorCh, 1:red 2:blue"},
        tooltip = "Use OpenCV Pipeline to scan for relative X location of a 'very' blue or red block"
    )
    public static void InitPropDetection(int colorChannel) {
        FindPropVisInitData visInitData = new FindPropVisInitData();
        visInitData.ColorChannel = colorChannel;
        visLib = new ExampleVisLib(
            hardwareMap,
            visInitData);
    }

    @ExportToBlocks 
    (
        comment = "By disabling pipeline, CPU usage can be potentially reduced"
    )
    public static void DisablePropDetection() {
        if (visLib == null)
            return;

        visLib.FindPropSetEnableDetection(false);
    }

    @ExportToBlocks (
        parameterLabels = {},
        comment = "Returns approximate X pixel location indicating how far left-to-right in the camera frame the most red/blue section was detected in",
        tooltip = "Returns approximate X pixel location indicating how far left-to-right in the camera frame the most red/blue section was detected in"
    )
    public static int GetPropXLocation () {
        if (visLib != null) {
            return visLib.GetPropXLocation();
        }
        return -1;
    }
}
