/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.RobotLog;

//import com.qualcomm.robotcore.hardware.DistanceSensor;

//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class intakeTest extends LinearOpMode{
    public NormalizedColorSensor Color = hardwareMap.get(NormalizedColorSensor.class, "color");
    //DistanceSensor Optical = hardwareMap.get(DistanceSensor.class, "color");


    public void runOpMode() throws InterruptedException {
        RobotLog.i("segment 1");
        waitForStart();
        RobotLog.i("segment 2");
        while(opModeIsActive()) {
            telemetryCode();
        }
    }

    public void telemetryCode() {
        telemetry.addData("A", "%.3f", Color.getNormalizedColors());
        //telemetry.addData("Dist", Optical.getDistance(DistanceUnit.INCH) + "IN");
        telemetry.update();
        RobotLog.i("segment 3");
    }
}
*/