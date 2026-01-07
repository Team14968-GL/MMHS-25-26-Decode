package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.lang.Math;

public class RoadRunnerAutoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Action action = drive.actionBuilder(new Pose2d(new Vector2d(0, 0), 0))
                .splineTo(new Vector2d(15, 15), Math.toRadians(180))
                .lineToX(10)
                .lineToY(20)
                .splineTo(new Vector2d(15, 15), 0)
                .splineTo(new Vector2d(-15, -15), 0)
                .lineToY(0)
                .lineToX(0)
                .setTangent(Math.PI/4)
                .build();

        waitForStart();


        if (isStopRequested()) {
            requestOpModeStop();
        }
    }
}