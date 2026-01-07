package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.lang.Math;

public class RoadRunnerAutoTest extends LinearOpMode {
        @Override
        public void runOpMode() {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(0,0), 0));
            Action action = drive.actionBuilder(new Pose2d(new Vector2d(0,0), 0))
                    .splineTo(new Vector2d(15,15), Math.toRadians(180))
                    .lineToX(10)
                    .lineToY(20)
                    .splineTo(new Vector2d(15,15),0)
                    .splineTo(new Vector2d(-15,-15), 0)
                    .lineToY(0)
                    .lineToX(0)
                    .build();

            waitForStart();

            if(isStopRequested()) return;

        }
    }

