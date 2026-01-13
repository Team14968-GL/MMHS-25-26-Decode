package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MMHS26Lib.debug;
import org.firstinspires.ftc.teamcode.MMHS26Lib.motion;
import org.firstinspires.ftc.teamcode.MMHS26Lib.utils;

@Autonomous
public class _26LibTest extends LinearOpMode{

    int time = 0;
    double speed = 0;

    @Override
    public void runOpMode(){
        debug.debugTelemetry = true;

        waitForStart();
        motion.moveForward(speed, time);
        utils.ledManager("Clear", 1);
        motion.moveBackward(speed, time);
        utils.ledManager("Good", 1);
        motion.strafeLeft(speed, time);
        utils.ledManager("Warn", 1);
        motion.strafeRight(speed, time);
        utils.ledManager("Alert", 1);
        motion.turnLeft(speed, time);
        utils.ledManager("Error", 1);
        motion.turnRight(speed, time);
        utils.ledManager("Match Alert", 1);
    }
}
