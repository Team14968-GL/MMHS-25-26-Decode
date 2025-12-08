package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp(name = "Starter OpMode")
public class ColorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        OpticalDistanceSensor Color = hardwareMap.get(OpticalDistanceSensor.class, "color");

        waitForStart();
        if (opModeIsActive()) {

        }
    }
}