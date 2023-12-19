package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;

public class BlueRedObjectDetectBlocks extends BlocksOpModeCompanion {
    static AutoCommon visLib = null;

    @ExportToBlocks (
        parameterLabels = {"colorCh, 1:red 2:blue"},
        tooltip = "Use OpenCV Pipeline to scan for the location of a 'very' blue or red block"
    )
    public static void InitPropDetection(int colorChannel) {
        FindPropVisInitData visInitData = new FindPropVisInitData();
        visInitData.ColorChannel = colorChannel;
        visLib = new AutoCommon(
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
        parameterLabels = {"ColorChannel"},
        comment = "Returns LEFT, MIDDLE or RIGHT String values",
        tooltip = "Returns LEFT, MIDDLE or RIGHT String values"
    )
    public static String PropLocation () {
        if (visLib != null) {
            return visLib.FindPropLocation();
        }
        return "ERROR: Prop detection not initialized or otherwise null object";
    }
}
