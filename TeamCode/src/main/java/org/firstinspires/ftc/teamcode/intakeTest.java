package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous
public class intakeTest extends LinearOpMode{
    ColorSensor Color = hardwareMap.get(ColorSensor.class, "color");
    DistanceSensor Optical = hardwareMap.get(DistanceSensor.class, "color");

    public void runOpMode() {
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("A", "%.3f", Color.alpha());
            telemetry.addData("R", "%.3f", Color.red());
            telemetry.addData("G", "%.3f", Color.green());
            telemetry.addData("B", "%.3f", Color.blue());
            telemetry.addData("Dist", Optical.getDistance(DistanceUnit.INCH) + "IN");
            telemetry.update();


        }
    }
}
