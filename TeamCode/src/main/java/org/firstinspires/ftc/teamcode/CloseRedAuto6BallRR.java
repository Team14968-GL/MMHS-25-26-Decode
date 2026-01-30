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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.lang.Math;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous
public class CloseRedAuto6BallRR extends LinearOpMode {
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
    private TouchSensor intakeBump1;
    private TouchSensor intakeBump2;

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

    ElapsedTime BackwardsTillBumpClock = new ElapsedTime();

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
        intakeBump1 = hardwareMap.get(TouchSensor.class, "intakeBump1");
        intakeBump2 = hardwareMap.get(TouchSensor.class, "intakeBump2");



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
        limelight.setPollRateHz(100);
        pinpoint.initialize();



        LLResultTypes.FiducialResult fiducialResult = null;
         scoop.setPosition(1);
        backDoor.setPosition(.5);
        turnTableServo.setPosition(0.5);
        goofyAhhhhFrontDoor.setPosition(0.5);



        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        Pose2d beginPose = new Pose2d(-52, 48, Math.toRadians(308));
        Pose2d PickUp1Pose = new Pose2d(-12, 52, Math.toRadians(90));
        Pose2d launchPose = new Pose2d(-24, 24,  Math.toRadians(308));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        TrajectoryActionBuilder MoveToScan = drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(-24, 24), Math.toRadians(22.5));

        TrajectoryActionBuilder MoveToLaunch = drive.actionBuilder(PickUp1Pose)
                .splineTo(new Vector2d(-24, 24), Math.toRadians(308));






                /*
                .lineToX(10)
                .lineToY(20)
                .splineTo(new Vector2d(15, 15), 0)
                .splineTo(new Vector2d(-15, -15), 0)
                .splineTo(new Vector2d(0,0),Math.PI/4);
                */


        TrajectoryActionBuilder Turn = drive.actionBuilder(beginPose)
                .turn(Math.toRadians(-11.25));


        TrajectoryActionBuilder PickUp1 = drive.actionBuilder(launchPose)
                .splineTo(new Vector2d(-16, 33), Math.toRadians(90));
        /*
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(-8, 35));

         */

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

        if (IDs.size() == 1) {
            Motif = IDs.get(0) - 21;
        } else if (IDs.size() == 2) {
            Motif = IDs.get(1) - 21;
        } else {
            Motif = 0;
        }
        launchMotif(Motif, launcherSpeed);
        sleep(250);
        goofyAhhhhFrontDoor.setPosition(1);
        backDoor.setPosition(.5);
        turnTableServo.setPosition(0);
        Actions.runBlocking(
                new SequentialAction(
                        PickUp1.build()));


        intake3Balls(.6, .5, .5, 400);



        Actions.runBlocking(
                new SequentialAction(
                        MoveToLaunch.build()));
        
        //localize();
        
        launchMotif(Motif, launcherSpeed);

        strafeLeftTics(1,ticPerIn*64);






    }
    public void strafeLeftTics(double Speed, double tic) {
        pinpoint.update();
        int yvalue = pinpoint.getEncoderY();
        while (yvalue - pinpoint.getEncoderY() <= tic) {
            pinpoint.update();
            leftBack.setPower(Speed);
            leftFront.setPower(-Speed);
            rightBack.setPower(-Speed);
            rightFront.setPower(Speed);
            telemetry.addData("yencoder", pinpoint.getEncoderY());
            telemetry.addData("yvalue", yvalue);
            telemetry.addData("y", pinpoint.getEncoderY()-yvalue);
            telemetry.update();
        }
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }
    public void strafeLeft(double Speed, int time) {
        leftBack.setPower(Speed);
        leftFront.setPower(-Speed);
        rightBack.setPower(-Speed);
        rightFront.setPower(Speed);
        sleep(time);
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    public void moveBackward(double Speed, int time) {
        leftBack.setPower(Speed);
        leftFront.setPower(Speed);
        rightBack.setPower(Speed);
        rightFront.setPower(Speed);
        sleep(time);
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }
    public void localize() {
        //boolean localizing = true;
        while (true) {
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
                if (!opModeIsActive()) {
                    break;
                } else if (tx < txMin) {
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
        int count = 0;
        while (count <= 5000) {
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
                    leftBack.setPower(-localPower);
                    leftFront.setPower(-localPower);
                    rightBack.setPower(localPower);
                    rightFront.setPower(localPower);
                    sleep(sleepTime);
                    leftBack.setPower(0);
                    leftFront.setPower(0);
                    rightBack.setPower(0);
                    rightFront.setPower(0);
                    count = count + sleepTime;
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
                    count = count + sleepTime;
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
                    count = count + sleepTime;
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
                    count = count + sleepTime;
                } else {
                    count++;
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
        telemetry.addData("done", 0);
        telemetry.update();
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
         scoop.setPosition(1);

        if ( Math.abs(motifArray.get(motiff*3) - motifArray.get((motiff*3)+1)) == 1){
            sleep(250);
        }

        launch(launcherSpeedd);
        turnTableServo.setPosition(motifArray.get((motiff*3)+2));
        sleep(500);
         scoop.setPosition(1);

        if ( Math.abs(motifArray.get((motiff*3)+1) - motifArray.get((motiff*3)+2)) == 1){
            sleep(250);
        }

        sleep(500);
        launch(launcherSpeedd);
        sleep(500);
         scoop.setPosition(1);
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
    public void intake3Balls(double searchSpeed, double returnSpeed, double returnDistance, int kickTime) {
        int safeTrig;
        turnTableServo.setPosition(0);
        goofyAhhhhFrontDoor.setPosition(1);
        intakeOn();
        safeTrig = BackwardsTillBump(searchSpeed,0);
        if (safeTrig == 1) {
            moveForwardTics(returnSpeed, returnDistance*ticPerIn);
            halfKick(kickTime);
            sleep(250);
            turnTableServo.setPosition(0.5);
            goofyAhhhhFrontDoor.setPosition(1);

            safeTrig = BackwardsTillBump(searchSpeed,0);
            if (safeTrig == 1) {
                moveForwardTics(returnSpeed, returnDistance*ticPerIn);
                halfKick(kickTime);
                sleep(250);
                turnTableServo.setPosition(1);
                goofyAhhhhFrontDoor.setPosition(1);
                safeTrig = BackwardsTillBump(searchSpeed,0);
                if (safeTrig == 1) {
                    halfKick(kickTime);
                    intakeOff();

                } else {

                    goofyAhhhhFrontDoor.setPosition(.5);
                    intakeOff();
                }

            } else {

                goofyAhhhhFrontDoor.setPosition(.5);
                intakeOff();
            }
        } else {

            goofyAhhhhFrontDoor.setPosition(.5);
            intakeOff();
        }


    }


    public void intakeOn() {
        intakeMotor.setPower(0.8);
    }

    public void intakeOff() {
        intakeMotor.setPower(0);

    }

    public int BackwardsTillBump(double Speed, int delay) {
        int count = 0;
        int returnSave = 2;
        BackwardsTillBumpClock.reset();
        while (BackwardsTillBumpClock.seconds() <= 2 && !(!intakeBump1.isPressed() || intakeBump2.isPressed())) {
            leftBack.setPower(Speed);
            leftFront.setPower(Speed);
            rightBack.setPower(Speed);
            rightFront.setPower(Speed);
            sleep(1);
            count++;
        }
        if (!intakeBump1.isPressed() || intakeBump2.isPressed()) {
            returnSave = 1;
        }

        if (BackwardsTillBumpClock.seconds() >= 2) {
            returnSave = 0;
        } else {
            returnSave = 1;
        }


        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        return returnSave;
    }
   
    public void moveForwardTics(double Speed, double tic) {
        pinpoint.update();
        double xvalue = pinpoint.getEncoderX();
        while (xvalue - pinpoint.getEncoderX() <= tic) {
            pinpoint.update();
            leftBack.setPower(-Speed);
            leftFront.setPower(-Speed);
            rightBack.setPower(-Speed);
            rightFront.setPower(-Speed);
        }
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }
    public void halfKick(int time) {
        goofyAhhhhFrontDoor.setPosition(0);
        sleep(time);
        goofyAhhhhFrontDoor.setPosition(.5);
    }
}



