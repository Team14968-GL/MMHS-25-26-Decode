package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MMHS26Lib.*;
import org.firstinspires.ftc.teamcode.MMHS26Lib.debug;
import org.firstinspires.ftc.teamcode.MMHS26Lib.motion;
import org.firstinspires.ftc.teamcode.MMHS26Lib.utils;
import org.firstinspires.ftc.teamcode.MMHS26Lib.roadRunner;

@Autonomous
public class _26LibTest extends LinearOpMode{
    long time = 1000;
    double speed = .5;
    Pose2d startingPose = new Pose2d(0,0,0);

    @Override
    public void runOpMode(){
        //setup
        new MMHS26Lib(hardwareMap);
        debug.debugTelemetry = true;

        waitForStart();
        //26Lib test script
        while(opModeIsActive()){
            telemetry.addData("X", MMHS26Lib.currentPose().position.x);
            telemetry.addData("Y", MMHS26Lib.currentPose().position.y);
            telemetry.addData("", MMHS26Lib.currentPose().heading.log());
            telemetry.update();
        }
    }
}
