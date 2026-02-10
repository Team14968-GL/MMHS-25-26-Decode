package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
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

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

@Autonomous
public class FarRedAuto3BallRR extends LinearOpMode {
    private GoBildaPinpointDriver pinpoint;

    Limelight3A limelight;

    public int id;
    public ArrayList<Integer> IDs = new ArrayList<>();
    //.5=green | 1=p | 0=p
    public ArrayList<Double> motifArray = new ArrayList<>(Arrays.asList(.5, 0.0, 1.0, 1.0, .5, 0.0, 1.0, 0.0, .5, 0.0));
    //public double[] motifArray = {.5, 0, 1, 1, .5, 0, 1, 0, .5};
    boolean telem = false;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor frontLeft;
    public DcMotor frontRight;
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


    double power = .7;
    double localPower = .3;
    double launcherSpeed = (2350 * 28) / 60.0;

    int sleepTime = 50;
    boolean processTrig = true;

    double xValue = 0;
    double ticPerIn = 254.7;
    double startX = (24 * 3) - (17.25 / 2);

    int Motif;
    int failCount;


    double txMax = 12.500;
    double txMin = 11.400;
    double tyMax = 13.5;
    double tyMin = 12;
    double taMax = .88;
    double taMin = .91;

    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!


        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");

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

        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        leftLauncher.setDirection(DcMotor.Direction.REVERSE);
        rightLauncher.setDirection(DcMotor.Direction.FORWARD);
        launchLiftRight.setDirection(CRServo.Direction.REVERSE);
        launchLiftLeft.setDirection(CRServo.Direction.FORWARD);
        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        scoop.setPosition(1);
        backDoor.setPosition(1);
        turnTableServo.setPosition(0.5);
        goofyAhhhhFrontDoor.setPosition(0.5);
        limelight.pipelineSwitch(0);
        pinpoint.initialize();

        LLResultTypes.FiducialResult fiducialResult = null;

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        Pose2d beginPose = new Pose2d(startX, 12, Math.toRadians(0));
        Pose2d leavePose = new Pose2d(49, 12, Math.toRadians(-20));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        new MMHS26Lib(hardwareMap, beginPose, telemetry);

        waitForStart();

        int count = 0;
        processTrig = true;
        while (count <= 1000 && processTrig) {
            processLimeLightResults();
            count++;
            sleep(1);
        }
        MMHS26Lib.roadRunner.turn(Math.toRadians(-20),
                MMHS26Lib.roadRunner.spline.splineToConstantHeading(startX * (1 - .125), 12, Math.toRadians(0), beginPose));

        if (localize(.3, 10)) {

            if (IDs.contains(21)) {
                Motif = 0;
                telemetry.addData("Motif", "GPP " + Motif);
            } else if (IDs.contains(22)) {
                Motif = 1;
                telemetry.addData("Motif", "PGP " + Motif);
            } else if (IDs.contains(23)) {
                Motif = 2;
                telemetry.addData("Motif", "PPG " + Motif);
            } else {
                Motif = 0;
                telemetry.addData("Motif", "Check failed " + Motif);
            }
            telemetry.update();

            telemetry.addData("IDs", IDs);
            telemetry.addData("Motif", Motif);
            telemetry.update();


            launchMotif(Motif, launcherSpeed, false);
            sleep(250);

            MMHS26Lib.roadRunner.spline.splineToLinearHeading(48, 48, Math.toRadians(0), Math.toRadians(0),  leavePose);
        }
    }

    public boolean localize(double localizerMotorPower, int sleepTimeMilli) {
        int count = 0;
        boolean finished = false;
        while (count <= 2000) {
            //boolean localizing = true;


            LLResult result = limelight.getLatestResult();
            double tx;
            double ty;
            double ta;
            if (result != null && result.isValid()) {
                tx = result.getTx(); // How far left or right the target is (degrees)
                ty = result.getTy(); // How far up or down the target is (degrees)
                ta = result.getTa(); // How big the target looks (0%-100% of the image)

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
                telemetry.update();

                if (tx < txMin) {
                    // turn right
                    telemetry.addData("Left", 0);
                    telemetry.update();
                    turnLeft(localizerMotorPower, sleepTimeMilli);
                    count = count + sleepTimeMilli;

                } else if (tx > txMax) {
                    //turn left
                    telemetry.addData("Right", 0);
                    telemetry.update();
                    turnRight(localizerMotorPower, sleepTimeMilli);

                    count = count + sleepTimeMilli;

                } else {
                    telemetry.addData("done", 0);
                    count = count + sleepTimeMilli;
                    telemetry.update();

                    finished = true;
                }
            } else {
                telemetry.addData("Limelight", "No Targets");
                // count = count + sleepTimeMilli;
                telemetry.update();
            }
        }
        return (finished);
    }

    public void turnRight(double Speed, int time) {
        backLeft.setPower(Speed);
        frontLeft.setPower(Speed);
        backRight.setPower(-Speed);
        frontRight.setPower(-Speed);
        sleep(time);
        backLeft.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
    }
    public void turnLeft(double Speed, int time) {
        backLeft.setPower(-Speed);
        frontLeft.setPower(-Speed);
        backRight.setPower(Speed);
        frontRight.setPower(Speed);
        sleep(time);
        backLeft.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
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
    private double filter(double input) {
        double output = 0.0;
        if (input == 0.5) {
            output = 0.0;
        } else if (input == 0.0) {
            output = 0.5;
        } else {
            output = input;
        }

        return output;
    }
    private void launchMotif(int motiff, double launcherSpeedd, boolean displacedGreen) {
        ArrayList<String> launchOrder = new ArrayList<>(Collections.emptyList());
        if (displacedGreen) {
            backDoor.setPosition(0);
            turnTableServo.setPosition(filter(motifArray.get(motiff * 3))); //motifArray.get(motiff*3)
            sleep(1000);
            launchMotorOn(launcherSpeedd);
            sleep(250);

            goofyAhhhhFrontDoor.setPosition(0);
            sleep(750);
            goofyAhhhhFrontDoor.setPosition(0.5);
            sleep(10);

            goofyAhhhhFrontDoor.setPosition(0.5);
            scoop.setPosition(0.5);

            turnTableServo.setPosition(filter(motifArray.get((motiff * 3) + 1))); //motifArray.get((motiff*3)+1)
            if (Math.abs(filter(motifArray.get(motiff * 3)) - filter(motifArray.get((motiff * 3) + 1))) == 1) {
                sleep(250);
            }
            sleep(500);
            scoop.setPosition(1);
            backDoor.setPosition(0);
            sleep(250);
            goofyAhhhhFrontDoor.setPosition(0);
            sleep(750);
            goofyAhhhhFrontDoor.setPosition(0.5);
            sleep(10);
            goofyAhhhhFrontDoor.setPosition(0.5);
            scoop.setPosition(0.5);

            turnTableServo.setPosition(filter(motifArray.get((motiff * 3) + 2)));
            if (Math.abs(filter(motifArray.get((motiff * 3) + 1)) - filter(motifArray.get((motiff * 3) + 2))) == 1) {
                sleep(250);
            }
            sleep(500);
            scoop.setPosition(1);
            backDoor.setPosition(0);
            sleep(250);
            goofyAhhhhFrontDoor.setPosition(0);
            sleep(750);
            goofyAhhhhFrontDoor.setPosition(0.5);
            sleep(200);

            goofyAhhhhFrontDoor.setPosition(0.5);
            scoop.setPosition(0.5);
            sleep(500);
            scoop.setPosition(1);
           backDoor.setPosition(1);
            turnTableServo.setPosition(0);
            launchMotorOff();
        } else {
            backDoor.setPosition(0);
            turnTableServo.setPosition(motifArray.get(motiff * 3)); //motifArray.get(motiff*3)
            sleep(1000);
            launchMotorOn(launcherSpeedd);
            sleep(250);

            goofyAhhhhFrontDoor.setPosition(0);
            sleep(750);
            goofyAhhhhFrontDoor.setPosition(0.5);
            sleep(10);

            goofyAhhhhFrontDoor.setPosition(0.5);
            scoop.setPosition(0.5);

            turnTableServo.setPosition(motifArray.get((motiff * 3) + 1)); //motifArray.get((motiff*3)+1)
            if (Math.abs(motifArray.get(motiff * 3) - motifArray.get((motiff * 3) + 1)) == 1) {
                sleep(250);
            }
            sleep(500);
            scoop.setPosition(1);
            backDoor.setPosition(0);
            sleep(250);
            goofyAhhhhFrontDoor.setPosition(0);
            sleep(750);
            goofyAhhhhFrontDoor.setPosition(0.5);
            sleep(10);
            goofyAhhhhFrontDoor.setPosition(0.5);
            scoop.setPosition(0.5);

            turnTableServo.setPosition(motifArray.get((motiff * 3) + 2));
            if (Math.abs(motifArray.get((motiff * 3) + 1) - motifArray.get((motiff * 3) + 2)) == 1) {
                sleep(250);
            }
            sleep(500);
            scoop.setPosition(1);
            backDoor.setPosition(0);
            sleep(250);
            goofyAhhhhFrontDoor.setPosition(0);
            sleep(750);
            goofyAhhhhFrontDoor.setPosition(0.5);
            sleep(200);

            goofyAhhhhFrontDoor.setPosition(0.5);
            scoop.setPosition(0.5);
            sleep(500);
            scoop.setPosition(1);
           backDoor.setPosition(1);
            turnTableServo.setPosition(0);
            launchMotorOff();
        }
    }

    private void launchMotif(int motiff, double launcherSpeedd) {
        ArrayList<String> launchOrder = new ArrayList<>(Collections.emptyList());
        if (motiff == 1) {
            launchMotorOn(launcherSpeedd);
            turnTableServo.setPosition(0);
            launchOrder.add("Purple");
            sleep(250);
            launch(launcherSpeedd);
            turnTableServo.setPosition(0.5);
            launchOrder.add("Green");

            sleep(500);
            scoop.setPosition(1);
                /*
        if ( Math.abs(motifArray.get(motiff*3) - motifArray.get((motiff*3)+1)) == 1){
            sleep(250);
        }
                */

            launch(launcherSpeedd);
            turnTableServo.setPosition(1);
            launchOrder.add("Purple");

            sleep(500);
            scoop.setPosition(1);
        /*
        if ( Math.abs(motifArray.get((motiff*3)+1) - motifArray.get((motiff*3)+2)) == 1){
            sleep(250);
        }
        */

            sleep(500);
            launch(launcherSpeedd);
            sleep(500);
            scoop.setPosition(1);
            launchMotorOff();

        } else if (motiff == 2) {
            launchMotorOn(launcherSpeedd);
            turnTableServo.setPosition(0);
            launchOrder.add("Purple");
            sleep(250);

            launch(launcherSpeedd);
            turnTableServo.setPosition(1);
            launchOrder.add("Purple");

            sleep(500);
            scoop.setPosition(1);
                /*
        if ( Math.abs(motifArray.get(motiff*3) - motifArray.get((motiff*3)+1)) == 1){
            sleep(250);
        }
                */

            launch(launcherSpeedd);
            turnTableServo.setPosition(0.5);
            launchOrder.add("Green");

            sleep(500);
            scoop.setPosition(1);
        /*
        if ( Math.abs(motifArray.get((motiff*3)+1) - motifArray.get((motiff*3)+2)) == 1){
            sleep(250);
        }
        */

            sleep(500);
            launch(launcherSpeedd);
            sleep(500);
            scoop.setPosition(1);
            launchMotorOff();
        } else {
            launchMotorOn(launcherSpeedd);
            turnTableServo.setPosition(.5);
            launchOrder.add("Green");
            sleep(250);

            launch(launcherSpeedd);
            turnTableServo.setPosition(0);
            launchOrder.add("Purple");

            sleep(500);
            scoop.setPosition(1);
                /*
        if ( Math.abs(motifArray.get(motiff*3) - motifArray.get((motiff*3)+1)) == 1){
            sleep(250);
        }
                */

            launch(launcherSpeedd);
            turnTableServo.setPosition(1);
            launchOrder.add("Purple");

            sleep(500);
            scoop.setPosition(1);
        /*
        if ( Math.abs(motifArray.get((motiff*3)+1) - motifArray.get((motiff*3)+2)) == 1){
            sleep(250);
        }
        */

            sleep(500);
            launch(launcherSpeedd);
            sleep(500);
            scoop.setPosition(1);
            launchMotorOff();

            telemetry.addData("launch order", launchOrder);
            telemetry.update();
        }

    }
    private void launch(double launcherSpeedd) {
        backDoor.setPosition(0);
        sleep(250);
        goofyAhhhhFrontDoor.setPosition(0);
        sleep(1250);
       backDoor.setPosition(1);
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
    public void localize() {
        //boolean localizing = true;
        int count = 0;
        while (count <= 3000) {
            LLResult result = limelight.getLatestResult();
            double tx ;
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
                    backLeft.setPower(-localPower);
                    frontLeft.setPower(-localPower);
                    backRight.setPower(localPower);
                    frontRight.setPower(localPower);
                    sleep(sleepTime);
                    backLeft.setPower(0);
                    frontLeft.setPower(0);
                    backRight.setPower(0);
                    frontRight.setPower(0);
                    count = count + sleepTime;
                } else if (tx > txMax) {
                    //turn left
                    backLeft.setPower(localPower);
                    frontLeft.setPower(localPower);
                    backRight.setPower(-localPower);
                    frontRight.setPower(-localPower);
                    sleep(sleepTime);
                    backLeft.setPower(0);
                    frontLeft.setPower(0);
                    backRight.setPower(0);
                    frontRight.setPower(0);
                    count = count + sleepTime;
                } else if (ta > taMax) {
                    //move backward
                    backLeft.setPower(localPower);
                    frontLeft.setPower(localPower);
                    backRight.setPower(localPower);
                    frontRight.setPower(localPower);
                    sleep(sleepTime);
                    backLeft.setPower(0);
                    frontLeft.setPower(0);
                    backRight.setPower(0);
                    frontRight.setPower(0);
                    count = count + sleepTime;
                } else if (ta < taMin) {
                    //move Forward
                    backLeft.setPower(-localPower);
                    frontLeft.setPower(-localPower);
                    backRight.setPower(-localPower);
                    frontRight.setPower(-localPower);
                    sleep(sleepTime);
                    backLeft.setPower(0);
                    frontLeft.setPower(0);
                    backRight.setPower(0);
                    frontRight.setPower(0);
                    count = count + sleepTime;
                } else {
                    count++;
                }

            } else {
                telemetry.addData("Limelight", "No Targets");
                telemetry.update();

                backLeft.setPower(0);
                frontLeft.setPower(0);
                backRight.setPower(0);
                frontRight.setPower(0);
            }
        }
        telemetry.addData("done", 0);
        telemetry.update();
    }




}