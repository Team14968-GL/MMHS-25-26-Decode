package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp(name = "Color Test")
public class ColorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        NormalizedColorSensor Color = hardwareMap.get(NormalizedColorSensor.class, "color");
        OpticalDistanceSensor Optical = hardwareMap.get(OpticalDistanceSensor.class, "color");

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.addData("OptLight", "%.3f", Optical.getLightDetected());
                NormalizedRGBA colors = Color.getNormalizedColors();
                telemetry.addData("NormAlpha", "%.3f", colors.alpha);
                telemetry.addData("NormRed", "%.3f", colors.red);
                telemetry.addData("NormGreen", "%.3f", colors.green);
                telemetry.addData("NormBlue", "%.3f", colors.blue);
                telemetry.update();
            }
        }
    }
}