package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Autonomous(name = "LimelightLocalizationTest")
public class LimelightLocalizationTest extends LinearOpMode {

    Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor frontLeft;
    public DcMotor frontRight;

    int sleepTime = 30;
    double power = .35;

    double txMax = 15;
    double txMin = 9;
    double tyMax = 13.5;
    double tyMin = 12;
    double taMax = 2.35;
    double taMin = 2.07;



    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");


        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);



        limelight.pipelineSwitch(0);
        pinpoint.initialize();


        LLResultTypes.FiducialResult fiducialResult = null;





        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");


        localize();

        double[] positon = localize();

        Pose2d localizePose = new Pose2d((positon[0] * 39.37), (positon[1] * 39.37), Math.toRadians(positon[2]));

        MecanumDrive drive = new MecanumDrive(hardwareMap, localizePose);




        TrajectoryActionBuilder launch = drive.actionBuilder(localizePose)
                .turnTo(Math.toRadians(308))
                .strafeToConstantHeading(new Vector2d(-18, 18));
                //.splineToLinearHeading(new Pose2d(new Vector2d(-18, 18), Math.toRadians(308)), 1);

        waitForStart();

        if (opModeIsActive()) {

            Actions.runBlocking(
                    new SequentialAction(
                            launch.build()));
            telemetry.addData("X",(positon[0] * 39.37));
            telemetry.addData("Y",(positon[1] * 39.37));
            telemetry.addData("Rotation",positon[2]);
            telemetry.addData("POSTION:", localizePose);
            telemetry.update();
            while(opModeIsActive());

        }


    }
    public double[] localize() {

        LLResult result = limelight.getLatestResult();
        double x = 0;
        double y = 0;
        double yaw = 0;

        Pose3D txx;

        if (result != null && result.isValid()) {

            txx = result.getBotpose();
             x = txx.getPosition().x;
             y = txx.getPosition().y;
             yaw = txx.getOrientation().getYaw();

            telemetry.addData("X", (txx.getPosition().x * 39.37));
            telemetry.addData("Y",  (txx.getPosition().y * 39.37));
            telemetry.addData("Rotation",  txx.getOrientation().getYaw());
            telemetry.update();

        } else {
            telemetry.addData("Limelight", "No Targets");
        }
        return new double[] {x,y,yaw};
    }
}