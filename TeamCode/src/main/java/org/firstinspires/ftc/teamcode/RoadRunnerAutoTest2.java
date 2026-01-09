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
        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        TrajectoryActionBuilder Move = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(10,0))
                .strafeTo(new Vector2d(10,10));
                //.splineTo(new Vector2d(24, 24), Math.toRadians(225))
                //.splineTo(new Vector2d(1,1), pinpoint.getHeading(AngleUnit.RADIANS));

        TrajectoryActionBuilder Move2 = drive.actionBuilder(beginPose);

        while (!isStopRequested() && !opModeIsActive());

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        Move.build(),
                        Move2.build()));
    }


}