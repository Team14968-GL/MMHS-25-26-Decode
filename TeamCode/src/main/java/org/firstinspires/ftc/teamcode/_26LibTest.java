package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MMHS26Lib.debug;
import org.firstinspires.ftc.teamcode.MMHS26Lib.motion;
import org.firstinspires.ftc.teamcode.MMHS26Lib.utils;
import org.firstinspires.ftc.teamcode.MMHS26Lib.roadRunner;

@Autonomous
public class _26LibTest extends LinearOpMode{
    int time = 500;
    double speed = 0.5;

    @Override
    public void runOpMode(){
        //setup
        new MMHS26Lib(hardwareMap);
        debug.debugTelemetry(true);

        waitForStart();
        //26Lib test script
        motion.moveForward(speed, time);
        utils.ledManager("Clear", 1);
        motion.moveBackward(speed, time);
        utils.ledManager("Good", 1);
        motion.strafeLeft(speed + 0.2, time);
        utils.ledManager("Warn", 1);
        motion.strafeRight(speed + 0.2, time);
        utils.ledManager("Alert", 1);
        motion.turnLeft(speed, time);
        utils.ledManager("Error", 1);
        motion.turnRight(speed, time);
        utils.ledManager("Match Alert", 1);
        roadRunner.spline.splineToSplineHeading(24, 24 , Math.toRadians(0), Math.toRadians(0), MMHS26Lib.currentPose());
    }
}
