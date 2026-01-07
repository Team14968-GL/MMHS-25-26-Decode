package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.lang.Math;

public class RoadRunnerAutoTest extends LinearOpMode {
        @Override
        public void runOpMode() {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(1,1), Math.PI));
            Action action = drive.actionBuilder(new Pose2d(new Vector2d(1,1), Math.PI/3))
                    .splineTo(new Vector2d(1,1), Math.PI)
                    .build();

            waitForStart();

            if(isStopRequested()) return;

        }
    }

