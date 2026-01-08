package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.lang.Math;

@Autonomous
public class RoadRunnerAutoTest2 extends LinearOpMode {
    private GoBildaPinpointDriver pinpoint;
    public void runOpMode() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        Pose2d beginPose = new Pose2d(52, 48, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        TrajectoryActionBuilder Move = drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(24, 24), Math.toRadians(225))
                .splineTo(new Vector2d(1,1), pinpoint.getHeading(AngleUnit.RADIANS));

                /*
                .lineToX(10)
                .lineToY(20)
                .splineTo(new Vector2d(15, 15), 0)
                .splineTo(new Vector2d(-15, -15), 0)
                .splineTo(new Vector2d(0,0),Math.PI/4);
                */

        TrajectoryActionBuilder Move2 = drive.actionBuilder(beginPose);

        while (!isStopRequested() && !opModeIsActive());

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        Move.build(),
                        Move2.build()));
    }


}