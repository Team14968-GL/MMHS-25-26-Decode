package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MMHS26Lib.*;

@Autonomous
public class _26LibTest extends LinearOpMode{
    long time = 1000;
    double speed = .5;
    final Pose2d startingPose = new Pose2d(0,0, 0);
    double X;
    double Y;
    double θ;

    @Override
    public void runOpMode(){
        //setup
        new MMHS26Lib(hardwareMap, startingPose, telemetry);
        debug.debugTelemetry = true;

        waitForStart();
        //26Lib test script
        while(opModeIsActive()){
            posUpdater();
            while (X <= -3 && opModeIsActive()){
                posUpdater();
                MMHS26Lib.motion.moveForward(0.3, 50);
            }
            while (X >= 3 && opModeIsActive()){
                posUpdater();
                MMHS26Lib.motion.moveBackward(0.3, 50);
            }
            while (θ >= 2.5 && opModeIsActive()){
                posUpdater();
                MMHS26Lib.motion.turnLeft(.3, 10);
            }
            while (θ <= -2.5 && opModeIsActive()){
                posUpdater();
                MMHS26Lib.motion.turnRight(.3, 10);
            }
        }
    }
    public void posUpdater(){
        X = MMHS26Lib.currentPose().position.x;
        Y = MMHS26Lib.currentPose().position.y;
        θ = Math.toDegrees(MMHS26Lib.currentPose().heading.toDouble());
        telemetry.addData("X", X);
        telemetry.addData("Y", Y);
        telemetry.addData("θ", θ);
        telemetry.update();
    }
}
