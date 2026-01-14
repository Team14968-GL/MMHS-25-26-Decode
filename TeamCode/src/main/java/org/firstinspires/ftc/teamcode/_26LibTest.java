package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MMHS26Lib.debug;
import org.firstinspires.ftc.teamcode.MMHS26Lib.motion;
import org.firstinspires.ftc.teamcode.MMHS26Lib.utils;

@Autonomous
public class _26LibTest extends LinearOpMode{

    int time = 500;
    double speed = 0.5;

    @Override
    public void runOpMode(){
        debug.debugTelemetry = true;

        waitForStart();
        motion.moveForward(speed, time, hardwareMap);
        utils.ledManager("Clear", 1, hardwareMap);
        motion.moveBackward(speed, time, hardwareMap);
        utils.ledManager("Good", 1, hardwareMap);
        motion.strafeLeft(speed + 0.2, time,hardwareMap);
        utils.ledManager("Warn", 1, hardwareMap);
        motion.strafeRight(speed + 0.2, time, hardwareMap);
        utils.ledManager("Alert", 1, hardwareMap);
        motion.turnLeft(speed, time, hardwareMap);
        utils.ledManager("Error", 1, hardwareMap);
        motion.turnRight(speed, time, hardwareMap);
        utils.ledManager("Match Alert", 1, hardwareMap);
    }
}
