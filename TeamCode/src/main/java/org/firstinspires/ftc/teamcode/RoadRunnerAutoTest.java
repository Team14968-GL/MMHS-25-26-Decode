package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class RoadRunnerAutoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(15, 15), Math.toRadians(180))
                        .lineToX(10)
                        .lineToY(20)
                        .splineTo(new Vector2d(15, 15), 0)
                        .splineTo(new Vector2d(-15, -15), 0)
                        .lineToY(0)
                        .lineToX(0)
                        .setTangent(Math.PI/4)
                        .build());
    }

}