package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.MMHS26Lib.debug;
import org.firstinspires.ftc.teamcode.MMHS26Lib.motion;
import org.firstinspires.ftc.teamcode.MMHS26Lib.utils;
import org.firstinspires.ftc.teamcode.MMHS26Lib.roadRunner;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "IntakeTest")
public class IntakeTest extends LinearOpMode {

    Limelight3A limelight;

    public int id;
    public ArrayList<Integer> IDs = new ArrayList<>();
    //.5=green | 1=p | 0=p
    public ArrayList<Double> motifArray = new ArrayList<>(Arrays.asList(.5, 0.0, 1.0, 1.0, .5, 0.0, 1.0, 0.0, .5));
    //public double[] motifArray = {.5, 0, 1, 1, .5, 0, 1, 0, .5};




    private DcMotor intakeMotor;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftLauncher;
    private DcMotor rightLauncher;
    private Servo backDoor;
    private Servo scoop;
    private Servo goofyAhhhhFrontDoor;
    private Servo turnTableServo;
    private GoBildaPinpointDriver pinpoint;
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
    double launcherSpeed = (1750 * 28) / 60;

    int sleepTime = 50;
    boolean processTrig = true;

    double xValue = 0;
    double ticPerIn = 254.7;


    int Motif;


    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftLauncher = hardwareMap.get(DcMotor.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotor.class, "rightLauncher");
        backDoor = hardwareMap.get(Servo.class, "backDoor");
        turnTableServo = hardwareMap.get(Servo.class, "turnTableServo");
        goofyAhhhhFrontDoor = hardwareMap.get(Servo.class, "goofyAhhhhFrontDoor");
        scoop = hardwareMap.get(Servo.class, "scoop");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        intakeBump1 = hardwareMap.get(TouchSensor.class, "intakeBump1");
        intakeBump2 = hardwareMap.get(TouchSensor.class, "intakeBump2");

        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        leftLauncher.setDirection(DcMotor.Direction.REVERSE);
        rightLauncher.setDirection(DcMotor.Direction.FORWARD);
        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        limelight.pipelineSwitch(0);
        pinpoint.initialize();

        LLResultTypes.FiducialResult fiducialResult = null;
        scoop.setPosition(0);
        backDoor.setPosition(1);
        turnTableServo.setPosition(0);
        goofyAhhhhFrontDoor.setPosition(1);

       // new MMHS26Lib(hardwareMap);



        waitForStart();

        if (opModeIsActive()) {
            intake3Balls(.3, .5, 1);


            //intake2Balls();
           // motion.moveForward(.5, 3000);
        }
    }
    public void intake3Balls(double searchSpeed, double returnSpeed, double returnDistance) {
        intakeOn();
        BackwardsTillBump(searchSpeed,0);
        moveForwardTics(returnSpeed, returnDistance*ticPerIn);
        halfKick();
        //sleep(250);
        turnTableServo.setPosition(0.5);
        goofyAhhhhFrontDoor.setPosition(1);

        BackwardsTillBump(searchSpeed,0);
        moveForwardTics(returnSpeed, returnDistance*ticPerIn);
        halfKick();
        //sleep(250);
        turnTableServo.setPosition(1);
        goofyAhhhhFrontDoor.setPosition(1);

        BackwardsTillBump(searchSpeed,0);
        moveForwardTics(returnSpeed, returnDistance*ticPerIn);
        halfKick();
        intakeOff();
    }


    public void intakeOn() {
        intakeMotor.setPower(0.8);
    }

    public void intakeOff() {
        intakeMotor.setPower(0);

    }
    public void BackwardsTillBump(double Speed, int delay) {
        int count = 0;
        while (count <= 2000 && !(!intakeBump1.isPressed() || intakeBump2.isPressed())) {
            leftBack.setPower(Speed);
            leftFront.setPower(Speed);
            rightBack.setPower(Speed);
            rightFront.setPower(Speed);
            sleep(1);
            count++;
        }
        if (count <= 2000) {
            sleep(delay);
        }


        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);



    }

    public void intake2Balls() {
        goofyAhhhhFrontDoor.setPosition(1);
        backDoor.setPosition(1);
        intakeMotor.setPower(0.8);
        turnTableServo.setPosition(0);
        moveBackward(.3, 1750);
        turnTableServo.setPosition(0.5);
        moveBackward(.3, 2000);
        turnTableServo.setPosition(1);
        moveBackward(.3, 1750);
        goofyAhhhhFrontDoor.setPosition(.5);
        sleep(500);
        goofyAhhhhFrontDoor.setPosition(0);
        sleep(500);
        goofyAhhhhFrontDoor.setPosition(.5);
        intakeMotor.setPower(0);
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
    public void halfKick() {
        goofyAhhhhFrontDoor.setPosition(0);
        sleep(750);
        goofyAhhhhFrontDoor.setPosition(.5);
    }
}