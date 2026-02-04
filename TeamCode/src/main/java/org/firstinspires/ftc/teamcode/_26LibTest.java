package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MMHS26Lib.*;

@SuppressWarnings({"NonAsciiCharacters", "unused", "InstantiationOfUtilityClass", "CommentedOutCode"})
@Config
@Autonomous
public class _26LibTest extends LinearOpMode{
    public static long time = 500;
    public static double speed = .7;
    final Pose2d startingPose = new Pose2d(0,0, 0);
    public static double X = .4;
    public static double Y = .7;
    public static double θ = 0;

    @Override
    public void runOpMode(){
        //setup
        new MMHS26Lib(hardwareMap, startingPose, telemetry);
        debug.debugTelemetry = true;

        waitForStart();
        //26Lib test script
        motion.mecanumDrive(X, Y, θ, speed, time);






//        while(opModeIsActive()){
//            posUpdater();
//            while (X <= -3 && opModeIsActive()){
//                posUpdater();
//                MMHS26Lib.motion.moveForward(speed, time);
//            }
//            while (X >= 3 && opModeIsActive()){
//                posUpdater();
//                MMHS26Lib.motion.moveBackward(speed, time);
//            }
//            while (θ >= 2.5 && opModeIsActive()){
//                posUpdater();
//                MMHS26Lib.motion.turnRight(speed - 0.2, time);
//            }
//            while (θ <= -2.5 && opModeIsActive()){
//                posUpdater();
//                MMHS26Lib.motion.turnLeft(speed - 0.2, time);
//            }
//        }
    }
//    public void posUpdater(){
//        X = MMHS26Lib.currentPose().position.x;
//        Y = MMHS26Lib.currentPose().position.y;
//        θ = Math.toDegrees(MMHS26Lib.currentPose().heading.toDouble());
//        telemetry.addData("X", X);
//        telemetry.addData("Y", Y);
//        telemetry.addData("θ", θ);
//        telemetry.update();
//    }
}
