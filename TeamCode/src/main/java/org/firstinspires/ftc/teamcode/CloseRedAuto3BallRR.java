package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.lang.Math;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous
public class CloseRedAuto3BallRR extends LinearOpMode {
    private GoBildaPinpointDriver pinpoint;

    Limelight3A limelight;

    public int id;
    public ArrayList<Integer> IDs = new ArrayList<>();
    //.5=green | 1=p | 0=p
    public ArrayList<Double> motifArray = new ArrayList<>(Arrays.asList(.5, 0.0, 1.0, 1.0, .5, 0.0, 1.0, 0.0, .5));
    //public double[] motifArray = {.5, 0, 1, 1, .5, 0, 1, 0, .5};
    boolean telem = false;

    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor intakeMotor;
    private DcMotor leftLauncher;
    private DcMotor rightLauncher;
    private Servo backDoor;
    private Servo scoop;
    private Servo goofyAhhhhFrontDoor;
    private Servo turnTableServo;
    private CRServo launchLiftRight;
    private CRServo launchLiftLeft;
    private TouchSensor TopBump;
    private TouchSensor BottomBump;


    double txMax = 15;
    double txMin = 9;
    double tyMax = 13.5;
    double tyMin = 12;
    double taMax = 2.35;
    double taMin = 2.07;

    double power = .7;
    double localPower = .3;
    double launcherSpeed = (1750.0 * 28.0) / 60;

    int sleepTime = 50;
    boolean processTrig = true;

    double xValue = 0;
    double ticPerIn = 254.7;


    int Motif;


    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); //

        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        leftLauncher = hardwareMap.get(DcMotor.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotor.class, "rightLauncher");
        backDoor = hardwareMap.get(Servo.class, "backDoor");
        turnTableServo = hardwareMap.get(Servo.class, "turnTableServo");
        goofyAhhhhFrontDoor = hardwareMap.get(Servo.class, "goofyAhhhhFrontDoor");
        scoop = hardwareMap.get(Servo.class, "scoop");
        TopBump = hardwareMap.get(TouchSensor.class, "TopBump");
        BottomBump = hardwareMap.get(TouchSensor.class, "BottomBump");
        launchLiftRight = hardwareMap.get(CRServo.class, "launchLiftRight");
        launchLiftLeft = hardwareMap.get(CRServo.class, "launchLiftLeft");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");



        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        leftLauncher.setDirection(DcMotor.Direction.REVERSE);
        rightLauncher.setDirection(DcMotor.Direction.FORWARD);
        launchLiftRight.setDirection(CRServo.Direction.REVERSE);
        launchLiftLeft.setDirection(CRServo.Direction.FORWARD);
        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight.pipelineSwitch(0);
        pinpoint.initialize();


        LLResultTypes.FiducialResult fiducialResult = null;
        scoop.setPosition(0);
        backDoor.setPosition(1);
        turnTableServo.setPosition(0.5);
        goofyAhhhhFrontDoor.setPosition(0.5);



        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        Pose2d beginPose = new Pose2d(-52, 48, Math.toRadians(308));
        Pose2d PickUp1Pose = new Pose2d(-12, 52, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        TrajectoryActionBuilder MoveToScan = drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(-24, 24), Math.toRadians(22.5));

        TrajectoryActionBuilder MoveToScan2 = drive.actionBuilder(PickUp1Pose)
                .splineTo(new Vector2d(-24, 24), Math.toRadians(11.25));



                /*
                .lineToX(10)
                .lineToY(20)
                .splineTo(new Vector2d(15, 15), 0)
                .splineTo(new Vector2d(-15, -15), 0)
                .splineTo(new Vector2d(0,0),Math.PI/4);
                */


        TrajectoryActionBuilder Turn = drive.actionBuilder(beginPose)
                .turn(Math.toRadians(-11.25));


        TrajectoryActionBuilder PickUp1 = drive.actionBuilder(beginPose)
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(-34, 68));

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        MoveToScan.build()));

        int count = 0;
        processTrig = true;
        while (count <= 1000 && processTrig) {
            processLimeLightResults();
            count++;
            sleep(1);
        }

        Actions.runBlocking(
                new SequentialAction(
                        Turn.build()));
        localize();

        if (IDs.size() == 1) {
            Motif = IDs.get(0) - 21;
        } else if (IDs.size() == 2) {
            Motif = IDs.get(0) - 21;
        } else {
            Motif = 0;
        }
        
        launchMotif(Motif, launcherSpeed);
        sleep(250);
        Actions.runBlocking(
                new SequentialAction(
                        PickUp1.build()));
        moveBackwardTics(.3,ticPerIn*3);


    }
    public void localize() {
        //boolean localizing = true;
        while (true) {
            LLResult result = limelight.getLatestResult();
            double tx;
            double ty;
            double ta;
            if (result != null && result.isValid()) {
                tx = result.getTx();
                ty = result.getTy(); // How far up or down the target is (degrees)
                ta = result.getTa(); // How big the target looks (0%-100% of the image)

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
                telemetry.update();

                if (tx < txMin) {
                    // turn right
                   leftBack.setPower(-localPower);
                   leftFront.setPower(-localPower);
                   rightBack.setPower(localPower);
                   rightFront.setPower(localPower);
                    sleep(sleepTime);
                   leftBack.setPower(0);
                   leftFront.setPower(0);
                   rightBack.setPower(0);
                   rightFront.setPower(0);
                } else if (tx > txMax) {
                    //turn left
                   leftBack.setPower(localPower);
                   leftFront.setPower(localPower);
                   rightBack.setPower(-localPower);
                   rightFront.setPower(-localPower);
                    sleep(sleepTime);
                   leftBack.setPower(0);
                   leftFront.setPower(0);
                   rightBack.setPower(0);
                   rightFront.setPower(0);
                } else if (ta > taMax) {
                    //move backward
                   leftBack.setPower(localPower);
                   leftFront.setPower(localPower);
                   rightBack.setPower(localPower);
                   rightFront.setPower(localPower);
                    sleep(sleepTime);
                   leftBack.setPower(0);
                   leftFront.setPower(0);
                   rightBack.setPower(0);
                   rightFront.setPower(0);
                } else if (ta < taMin) {
                    //move Forward
                   leftBack.setPower(-localPower);
                   leftFront.setPower(-localPower);
                   rightBack.setPower(-localPower);
                   rightFront.setPower(-localPower);
                    sleep(sleepTime);
                   leftBack.setPower(0);
                   leftFront.setPower(0);
                   rightBack.setPower(0);
                   rightFront.setPower(0);
                } else {
                    break;
                }

            } else {
                telemetry.addData("Limelight", "No Targets");
                telemetry.update();

               leftBack.setPower(0);
               leftFront.setPower(0);
               rightBack.setPower(0);
               rightFront.setPower(0);
            }
        }
    }
    public void moveBackwardTics(double Speed, double tic) {
        pinpoint.update();
        int xvalue = pinpoint.getEncoderX(); // \/
        while (pinpoint.getEncoderX() - xvalue <= tic) {
            pinpoint.update();
            leftBack.setPower(Speed);
            leftFront.setPower(Speed);
            rightBack.setPower(Speed);
            rightFront.setPower(Speed);
            telemetry.addData("xencoder", pinpoint.getEncoderX());
            telemetry.addData("xvalue", xvalue);
            telemetry.update();
        }
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }


    private int processLimeLightResults() {
        double tx;
        double ty;
        double ta;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            // Get the list of ALL detected fiducials (AprilTags)
            List<LLResultTypes.FiducialResult> fiducialList = result.getFiducialResults();

            tx = result.getTx();
            ty = result.getTy(); // How far up or down the target is (degrees)
            ta = result.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
            telemetry.update();

            if (!fiducialList.isEmpty()) {
                telemetry.addData("Detections Found", fiducialList.size());
                telemetry.update();

                // Iterate through each detected tag
                for (LLResultTypes.FiducialResult fiducial : fiducialList) {
                    id = fiducial.getFiducialId();
                    IDs.add(id);
                    telemetry.addData("Tag ID", id);
                    telemetry.update();
                }
                processTrig = false;
            } else {
                telemetry.addData("Detections Found", "None");
                telemetry.update();
            }
        } else {
            telemetry.addData("Limelight Data", "Invalid or Stale");
            assert result != null;
            telemetry.addData("Staleness", result.getStaleness());
            telemetry.update();
        }
        telemetry.update();

        return id;
    }

    private void launchMotif(int motiff, double launcherSpeedd) {
        launchMotorOn(launcherSpeedd);
        turnTableServo.setPosition(motifArray.get(motiff*3));
        sleep(250);
        launch(launcherSpeedd);
        turnTableServo.setPosition(motifArray.get((motiff*3)+1));
        sleep(500);
        scoop.setPosition(0);
                /*
        if ( Math.abs(motifArray.get(motiff*3) - motifArray.get((motiff*3)+1)) == 1){
            sleep(250);
        }
                */

        launch(launcherSpeedd);
        turnTableServo.setPosition(motifArray.get((motiff*3)+2));
        sleep(500);
        scoop.setPosition(0);
        /*
        if ( Math.abs(motifArray.get((motiff*3)+1) - motifArray.get((motiff*3)+2)) == 1){
            sleep(250);
        }
        */

        sleep(500);
        launch(launcherSpeedd);
        sleep(500);
        scoop.setPosition(0);
        launchMotorOff();
    }
    private void launch(double launcherSpeedd) {
        backDoor.setPosition(0);
        sleep(250);
        goofyAhhhhFrontDoor.setPosition(0);
        sleep(1250);
        backDoor.setPosition(0.5);
        goofyAhhhhFrontDoor.setPosition(0.5);
        scoop.setPosition(0.5);
    }
    private void launchMotorOn(double launcherSpeedd) {
        ((DcMotorEx) leftLauncher).setVelocity(launcherSpeedd);
        ((DcMotorEx) rightLauncher).setVelocity(launcherSpeedd);
    }
    private void launchMotorOff() {
        ((DcMotorEx) leftLauncher).setVelocity(0);
        ((DcMotorEx) rightLauncher).setVelocity(0);
    }




}