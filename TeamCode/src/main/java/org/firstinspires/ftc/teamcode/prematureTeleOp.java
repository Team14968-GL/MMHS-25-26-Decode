
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "prematureTeleOp")
public class prematureTeleOp extends LinearOpMode {

    Limelight3A limelight;

    private DcMotor intakeMotor;
    private Servo goofyAhhhhFrontDoor;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftLauncher;
    private DcMotor rightLauncher;
    private DcMotor lift;
    private CRServo launchLiftRight;
    private CRServo launchLiftLeft;
    private Servo backDoor;
    private Servo scoop;
    private Servo turnTableServo;
    private TouchSensor TopBump;
    private TouchSensor BottomBump;
    private DistanceSensor distance;
    private DistanceSensor color_DistanceSensor;
    private CRServo LED1;
    private GoBildaPinpointDriver pinpoint;

    int highLauncherSpeed = 2165;
    int lowLauncherSpeed = 1700;
    int triangleFuncRunning = 0;
    double turnTablePos2 = 0;
    int launcherSpeed = 0;
    double speed = 0;




    ElapsedTime triangleClock = new ElapsedTime();
    ElapsedTime ReKickClock = new ElapsedTime();
    ElapsedTime ScoopClock = new ElapsedTime();
    int RekickTrig = 0;
    int scoopTrig = 0;
    int triTrig = 0;
    int LocalTrig = 0;
    int ledTrig = 0;
    int bumpTrig = 0;

    double txMax = 14.700;
    double txMin = 14.400;
    double tyMax = 13.5;
    double tyMin = 12;
    double taMax = .88;
    double taMin = .91;

    @Override
    public void runOpMode() {

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        goofyAhhhhFrontDoor = hardwareMap.get(Servo.class, "goofyAhhhhFrontDoor");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftLauncher = hardwareMap.get(DcMotor.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotor.class, "rightLauncher");
        lift = hardwareMap.get(DcMotor.class, "lift");
        launchLiftRight = hardwareMap.get(CRServo.class, "launchLiftRight");
        launchLiftLeft = hardwareMap.get(CRServo.class, "launchLiftLeft");
        backDoor = hardwareMap.get(Servo.class, "backDoor");
        scoop = hardwareMap.get(Servo.class, "scoop");
        turnTableServo = hardwareMap.get(Servo.class, "turnTableServo");
        TopBump = hardwareMap.get(TouchSensor.class, "TopBump");
        BottomBump = hardwareMap.get(TouchSensor.class, "BottomBump");
        distance = hardwareMap.get(DistanceSensor.class, "distance");
        color_DistanceSensor = hardwareMap.get(DistanceSensor.class, "color");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        LED1 = hardwareMap.get(CRServo.class, "Led1");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");


        // Put initialization blocks here.
        triangleFuncRunning = 0;
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        leftLauncher.setDirection(DcMotor.Direction.REVERSE);
        rightLauncher.setDirection(DcMotor.Direction.FORWARD);
        launchLiftRight.setDirection(CRServo.Direction.REVERSE);
        launchLiftLeft.setDirection(CRServo.Direction.FORWARD);
        leftLauncher.setPower(0);
        rightLauncher.setPower(0);
        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();
        ledManager("Null");

        waitForStart();
        speed = 0.75;
        backDoor.setPosition(1);
        goofyAhhhhFrontDoor.setPosition(0.5);
        scoop.setPosition(0);
        turnTableServo.setPosition(0.5);
        turnTablePos2 = 0;
        launcherSpeed = (1700 * 28) / 60;
        triangleFuncRunning = 1;

        pinpoint.initialize();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                /*
                if (ledTrig == 0) {
                    ledManager("Null");
                }

                 */
                telemetry.update();
                pinpoint.update();
                telemetry.addData("x", pinpoint.getEncoderX());
                telemetry.addData("y", pinpoint.getEncoderY());
                telemetry.addData("posX", pinpoint.getPosX(DistanceUnit.MM));
                telemetry.addData("posY", pinpoint.getPosY(DistanceUnit.MM));
                telemetry.addData("headingDeg", pinpoint.getHeading(AngleUnit.DEGREES));
                intakeControl();
                drive();
                turnTablePos();
                timeTriangleFunction();
                backDoorControl();
                timeReKick();
                timeScoop();
                controlLauncher();
                goofyAhhhhFrontDoorControl();
                launcherTiltControl();
                //distanceSensorControl();
                killSwitch();
                localize(0.3, 10);
                lift();
            }
        }
    }


    private void intakeControl() {
        if (gamepad1.left_trigger == 1) {
            intakeMotor.setPower(0.8);
            goofyAhhhhFrontDoor.setPosition(1);
        } else if (gamepad1.right_trigger == 1) {
            intakeMotor.setPower(0);
        }
    }

    private void backDoorControl() {
        if (gamepad2.squareWasPressed()) {
            backDoor.setPosition(1);
        }
        if (gamepad2.circleWasPressed()) {
            backDoor.setPosition(0);
        }
    }

    private void launcherTiltControl() {

        if (-0.1 >= gamepad2.right_stick_y && !BottomBump.isPressed()) {
            launchLiftRight.setPower(gamepad2.right_stick_y * 0.35);
            launchLiftLeft.setPower(gamepad2.right_stick_y * 0.35);
        } else if (0.1 <= gamepad2.right_stick_y && !TopBump.isPressed()) {
            launchLiftRight.setPower(gamepad2.right_stick_y * 0.35);
            launchLiftLeft.setPower(gamepad2.right_stick_y * 0.35);
        } else if (0.1 <= gamepad2.right_stick_y && TopBump.isPressed()) {
            launchLiftRight.setPower(0);
            launchLiftRight.setPower(0);
            launchLiftLeft.setPower(0);
            gamepad2.rumbleBlips(1);
        } else if (-0.1 >= gamepad2.right_stick_y && BottomBump.isPressed()) {
            launchLiftRight.setPower(0);
            launchLiftLeft.setPower(0);
            gamepad2.rumbleBlips(1);
        } else {
            launchLiftRight.setPower(0);
            launchLiftLeft.setPower(0);
        }
        if (TopBump.isPressed()) {
            ledManager("Blue");
            bumpTrig = 1;
        } else if  (BottomBump.isPressed()) {
            ledManager("Blue");
            bumpTrig = 1;
        }else if (bumpTrig == 1 && !(BottomBump.isPressed() || TopBump.isPressed())){
            ledManager("Null");
            bumpTrig = 0;
        }
    }

    private void goofyAhhhhFrontDoorControl() {
        if (gamepad1.squareWasReleased()) {
            goofyAhhhhFrontDoor.setPosition(1);
        } else if (gamepad1.circleWasReleased()) {
            goofyAhhhhFrontDoor.setPosition(0);
        } else if (gamepad1.crossWasReleased()) {
            goofyAhhhhFrontDoor.setPosition(0.5);
        }
    }

    private void killSwitch() {
        if (gamepad1.touchpadWasPressed()) {
            leftLauncher.setPower(0);
            rightLauncher.setPower(0);
            intakeMotor.setPower(0);
        }
    }

    private void distanceSensorControl() {
        if (distance.getDistance(DistanceUnit.CM) <= 0) {
            opModeIsActive(); //this is a useless function
        }
        telemetry.addData("Distance", color_DistanceSensor.getDistance(DistanceUnit.CM));
    }

    private void turnTablePos() {
        if (gamepad2.leftBumperWasPressed() && 0 != goofyAhhhhFrontDoor.getPosition()) {
            turnTablePos2 += 0.5;
            if (1.5 <= turnTablePos2) {
                turnTablePos2 = 0;
            }
            turnTableServo.setPosition(turnTablePos2);
        }
        if (gamepad2.rightBumperWasPressed() && 0 != goofyAhhhhFrontDoor.getPosition()) {
            turnTablePos2 -= 0.5;
            if (-0.5 >= turnTablePos2) {
                turnTablePos2 = 1;
            }
            turnTableServo.setPosition(turnTablePos2);
        }
    }

    private void drive() {
        @SuppressWarnings("SuspiciousNameCombination") float ControlY = gamepad1.left_stick_x;
        float ControlX = -gamepad1.right_stick_x;
        float ControlRX = -gamepad1.right_stick_y;
        leftFront.setPower(((ControlY - ControlX) + ControlRX) * speed);
        leftBack.setPower((ControlY + ControlX + ControlRX) * speed);
        rightFront.setPower(((ControlY - ControlX) - ControlRX) * speed);
        rightBack.setPower(((ControlY + ControlX) - ControlRX) * speed);
        if (gamepad1.dpad_up) {
            speed = 1;
        } else if (gamepad1.dpad_right) {
            speed = 0.75;
        } else if (gamepad1.dpad_down) {
            speed = 0.35;
        }
    }

    private void timeReKick() {


        if (gamepad2.touchpadWasReleased()) {
            ReKickClock.reset();
            telemetry.addData("Elapsed Time", ReKickClock.seconds());
            RekickTrig = 1;
        }
        if (RekickTrig == 1) {
            if (ReKickClock.seconds() >= 0 && ReKickClock.seconds() <= 0.75) {
                goofyAhhhhFrontDoor.setPosition(0);
                telemetry.update();
            }
            if (ReKickClock.seconds() >= 0.75 && ReKickClock.seconds() <= 1) {
                goofyAhhhhFrontDoor.setPosition(0.5);
                telemetry.update();
                RekickTrig = 0;
            }
        }
    }

    private void timeTriangleFunction() {

        if (gamepad2.triangleWasReleased()) {
            triangleFuncRunning = 1;

            triangleClock.reset();
            telemetry.addData("Elapsed Time", triangleClock.seconds());
            triTrig = 1;
        }
        if (triTrig == 1) {
            if (triangleClock.seconds() >= 0 && triangleClock.seconds() <= 0.5) {
                triangleFuncRunning = 0;
                launchMotorOnTriangle();
                backDoor.setPosition(0);
                telemetry.update();
                ledManager("Alert");
                ledTrig = 1;
            }
            if (triangleClock.seconds() >= 0.5 && triangleClock.seconds() <= 1.5) {
                launchMotorOnTriangle();
                goofyAhhhhFrontDoor.setPosition(0);
                telemetry.update();
            }
            if (triangleClock.seconds() >= 1.5 && triangleClock.seconds() <= 2) {
                launchMotorOnTriangle();
                goofyAhhhhFrontDoor.setPosition(0.5);
                scoop.setPosition(0.5);
                telemetry.update();
            }
            if (triangleClock.seconds() >= 2.5 && triangleClock.seconds() <= 3) {
                launchMotorOnTriangle();
                scoop.setPosition(0);
                telemetry.update();
                triangleFuncRunning = 0;
                triTrig = 0;
                ledManager("Null");
                ledTrig = 0;
            }

        }
    }

    private void timeScoop() {

        if (gamepad2.dpad_up) {
            ScoopClock.reset();
            telemetry.addData("Elapsed Time", ScoopClock.seconds());
            scoopTrig = 1;
        }
        if (scoopTrig == 1) {
            if (ScoopClock.seconds() >= 0 && ScoopClock.seconds() <= 0.5) {
                scoop.setPosition(0.5);
                telemetry.update();
            }
            if (ScoopClock.seconds() >= 0.5 && ScoopClock.seconds() <= 1) {
                scoop.setPosition(0);
                telemetry.update();
                scoopTrig = 0;
            }
        }
    }

    private void controlLauncher() {
        int LauncherON = 0;

        if (gamepad2.left_trigger == 1) {
            ((DcMotorEx) leftLauncher).setVelocity(launcherSpeed);
            ((DcMotorEx) rightLauncher).setVelocity(launcherSpeed);
            LauncherON = 1;
        } else if (gamepad2.right_trigger == 1) {
            ((DcMotorEx) leftLauncher).setVelocity(0);
            ((DcMotorEx) rightLauncher).setVelocity(0);
            LauncherON = 0;
        }
        if (gamepad2.dpadLeftWasPressed()) {
            launcherSpeed = (lowLauncherSpeed * 28) / 60;
            gamepad2.rumbleBlips(1);
            if (1 == LauncherON) {
                ((DcMotorEx) leftLauncher).setVelocity(launcherSpeed);
                ((DcMotorEx) rightLauncher).setVelocity(launcherSpeed);
            }
        } else if (gamepad2.dpadRightWasReleased()) {
            launcherSpeed = (highLauncherSpeed * 28) / 60;
            gamepad2.rumbleBlips(2);
            if (1 == LauncherON) {
                ((DcMotorEx) leftLauncher).setVelocity(launcherSpeed);
                ((DcMotorEx) rightLauncher).setVelocity(launcherSpeed);
            }
        }
    }

    private void launchMotorOnTriangle() {
        ((DcMotorEx) leftLauncher).setVelocity(launcherSpeed * Math.abs(triangleFuncRunning - 1));
        ((DcMotorEx) rightLauncher).setVelocity(launcherSpeed * Math.abs(triangleFuncRunning - 1));
    }
    public void localize(double localizerMotorPower, int sleepTimeMilli) {
        if (gamepad1.ps) {
            LocalTrig = 1;
            //boolean localizing = true;

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
                    leftBack.setPower(-localizerMotorPower);
                    leftFront.setPower(-localizerMotorPower);
                    rightBack.setPower(-localizerMotorPower);
                    rightFront.setPower(-localizerMotorPower);
                    sleep(sleepTimeMilli);
                    leftBack.setPower(0);
                    leftFront.setPower(0);
                    rightBack.setPower(0);
                    rightFront.setPower(0);
                    ledManager("Clear");
                    LocalTrig = 1;
                } else if (tx > txMax) {
                    //turn left
                    leftBack.setPower(localizerMotorPower);
                    leftFront.setPower(localizerMotorPower);
                    rightBack.setPower(localizerMotorPower);
                    rightFront.setPower(localizerMotorPower);
                    sleep(sleepTimeMilli);
                    leftBack.setPower(0);
                    leftFront.setPower(0);
                    rightBack.setPower(0);
                    rightFront.setPower(0);
                    ledManager("Clear");
                    LocalTrig = 1;
                } else if (ta > taMax) {
                    //move backward
                    leftBack.setPower(localizerMotorPower);
                    leftFront.setPower(localizerMotorPower);
                    rightBack.setPower(-localizerMotorPower);
                    rightFront.setPower(-localizerMotorPower);
                    sleep(sleepTimeMilli);
                    leftBack.setPower(0);
                    leftFront.setPower(0);
                    rightBack.setPower(0);
                    rightFront.setPower(0);
                    ledManager("Clear");
                    LocalTrig = 1;
                } else if (ta < taMin) {
                    //move Forward
                    leftBack.setPower(-localizerMotorPower);
                    leftFront.setPower(-localizerMotorPower);
                    rightBack.setPower(localizerMotorPower);
                    rightFront.setPower(localizerMotorPower);
                    sleep(sleepTimeMilli);
                    leftBack.setPower(0);
                    leftFront.setPower(0);
                    rightBack.setPower(0);
                    rightFront.setPower(0);
                    ledManager("Clear");
                    LocalTrig = 1;
                } else {
                    telemetry.addData("done", 0);
                    ledManager("Good");
                    LocalTrig = 1;
                }

            } else {
                telemetry.addData("Limelight", "No Targets");
                ledManager("Error");
                LocalTrig = 1;
                leftBack.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
            }
        } else if (LocalTrig == 1){
            ledManager("Null");
            LocalTrig = 0;
        }
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
    public void strafeRight(double Speed, int time) {
        leftBack.setPower(-Speed);
        leftFront.setPower(Speed);
        rightBack.setPower(Speed);
        rightFront.setPower(-Speed);
        sleep(time);
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }
    public void turnRight(double Speed, int time) {
        leftBack.setPower(Speed);
        leftFront.setPower(Speed);
        rightBack.setPower(-Speed);
        rightFront.setPower(-Speed);
        sleep(time);
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }
    public void turnLeft(double Speed, int time) {
        leftBack.setPower(-Speed);
        leftFront.setPower(-Speed);
        rightBack.setPower(Speed);
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
    public void moveForward(double Speed, int time) {
        leftBack.setPower(-Speed);
        leftFront.setPower(-Speed);
        rightBack.setPower(-Speed);
        rightFront.setPower(-Speed);
        sleep(time);
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }
    private void lift(){
        lift.setPower(gamepad2.left_stick_y);
        /*
        if (!liftLimit.isPressed()) {
            lift.setPower(gamepad2.right_stick_y);
        } else {
            lift.setPower(0);
            ledManager("Good");
        }

         */

    }
    private void ledManager(String type){
        switch (type) {
            case "Clear":
                LED1.setPower(.5); //White

                break;
            case "Good":
                LED1.setPower(0); //Green

                break;
            case "Warn":
                LED1.setPower(-.25); //Yellow

                break;
            case "Alert":
                LED1.setPower(-.35); //Orange

                break;
            case "Error":
                LED1.setPower(-0.44); //red

                break;
            case "Null":
                LED1.setPower(-.6); //Blank

                break;
            case "Match Alert":
                LED1.setPower(0); //Purple

                break;
            case "Blue":
                LED1.setPower(0.216); //Blue

                break;
            case "Purple":
                LED1.setPower(0.415); //Purple

                break;
            case "Pink":
                LED1.setPower(0.275); //Pink

                break;
            default:
                telemetry.addData("Led Manager Error", "Wrong or Invalid Input");
                break;
        }
    }
}
