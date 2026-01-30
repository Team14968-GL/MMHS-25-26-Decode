package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.MMHS26Lib.debug;
import org.firstinspires.ftc.teamcode.MMHS26Lib.motion;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class ME_Suffering extends LinearOpMode {
    int time = 650;
    double speed = 1;

    Pose2d startingPose = new Pose2d(0, 0, 0);

    @Override
    public void runOpMode() {
        //setup
        //noinspection InstantiationOfUtilityClass
        new MMHS26Lib(hardwareMap);
        debug.debugTelemetry = true;

        waitForStart();
        //26Lib test script
        motion.moveForward(speed, time);
}}
