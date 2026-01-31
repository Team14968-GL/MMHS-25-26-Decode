package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import java.util.ArrayList;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "CloseRedLLAutoTime")
public class CloseRedLLAutoTime extends LinearOpMode {

    Limelight3A limelight;

    public int id;
    public ArrayList<Integer> IDs = new ArrayList<>();
    //.5=green | 1=p | 0=p
    public ArrayList<Double> motifArray = new ArrayList<>(Arrays.asList(.5, 0.0, 1.0, 1.0, .5, 0.0, 1.0, 0.0, .5));
    //public double[] motifArray = {.5, 0, 1, 1, .5, 0, 1, 0, .5};
    boolean telem = false;
    public  DcMotor backLeft;
    public  DcMotor backRight;
    public  DcMotor frontLeft;
    public DcMotor frontRight;
    private DcMotor leftLauncher;
    private DcMotor rightLauncher;
    private Servo backDoor;
    private Servo scoop;
    private Servo goofyAhhhhFrontDoor;
    private Servo turnTableServo;
    private GoBildaPinpointDriver pinpoint;




    double txMax = 15;
    double txMin = 9;
    double tyMax = 13.5;
    double tyMin = 12;
    double taMax = 2.35;
    double taMin = 2.07;

    double power = .5;
    double localPower = .3;
    double launcherSpeed = (1750 * 28) / 60.0;

    int sleepTime = 50;
    boolean processTrig = true;

    double xValue = 0;


    int Motif;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
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
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");


        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        leftLauncher.setDirection(DcMotor.Direction.REVERSE);
        rightLauncher.setDirection(DcMotor.Direction.FORWARD);
        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight.pipelineSwitch(0);
        pinpoint.initialize();

        LLResultTypes.FiducialResult fiducialResult = null;
        scoop.setPosition(1);
        backDoor.setPosition(.5);
        turnTableServo.setPosition(0.5);
        goofyAhhhhFrontDoor.setPosition(0.5);



        waitForStart();

        if (opModeIsActive()) {
            xValue = pinpoint.getEncoderX();
            moveBackward(power,1000);
            sleep(500);
            turnLeft(power, 500);

            int count = 0;
            processTrig = true;
            while (count <= 1000 && processTrig) {
                processLimeLightResults();
                count++;
                sleep(1);
            }

            telemetry.addData("IDs size", IDs.size());
            telemetry.addData("-",IDs);
            telemetry.update();

            sleep(500);
            turnRight(power,500);

            sleep(500);
            localize();
            if (IDs.size() == 1) {
                Motif = IDs.get(0) - 21;
            } else if (IDs.size() == 2) {
                Motif = IDs.get(0) - 21;
            } else {
                Motif = 0;
            }
            launchMotif(Motif, launcherSpeed);
            strafeLeft(.6, 1000);
            scoop.setPosition(1);
            backDoor.setPosition(0);
            turnTableServo.setPosition(0.5);
            goofyAhhhhFrontDoor.setPosition(0.5);
        }
    }
    private void launchMotif(int motiff, double launcherSpeedd) {
        launchMotorOn(launcherSpeedd);
        turnTableServo.setPosition(motifArray.get(motiff*3));
        sleep(500);
        launch(launcherSpeedd);
        turnTableServo.setPosition(motifArray.get((motiff*3)+1));
        sleep(500);
        launch(launcherSpeedd);
        turnTableServo.setPosition(motifArray.get((motiff*3)+2));
        sleep(500);
        launch(launcherSpeedd);
        launchMotorOff();



    }
    private void launch(double launcherSpeedd) {
        backDoor.setPosition(0);
        sleep(500);
        goofyAhhhhFrontDoor.setPosition(0);
        sleep(1000);
        goofyAhhhhFrontDoor.setPosition(0.5);
        scoop.setPosition(0.5);
        sleep(1000);
        scoop.setPosition(1);
    }
    private void launchMotorOn(double launcherSpeedd) {
        ((DcMotorEx) leftLauncher).setVelocity(launcherSpeedd);
        ((DcMotorEx) rightLauncher).setVelocity(launcherSpeedd);
    }
    private void launchMotorOff() {
        ((DcMotorEx) leftLauncher).setVelocity(0);
        ((DcMotorEx) rightLauncher).setVelocity(0);
    }

    private int processLimeLightResults() {
        double tx;
        double ty;
        double ta;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            //gets the list of detected AprilTags
            List<LLResultTypes.FiducialResult> fiducialList = result.getFiducialResults();

            tx = result.getTx(); //degrees from y-axis
            ty = result.getTy(); //degrees from x-axis
            ta = result.getTa(); //% of image that the tag covers

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
            telemetry.update();

            if (!fiducialList.isEmpty()) {
                telemetry.addData("Detections Found", fiducialList.size());
                telemetry.update();

                //iterates through each detected tag
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
    public void strafeLeft(double Speed, int time) {
        backLeft.setPower(Speed);
        frontLeft.setPower(-Speed);
        backRight.setPower(-Speed);
        frontRight.setPower(Speed);
        sleep(time);
        backLeft.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
    }
    public void strafeLeftTics(double Speed, int tic) {
        pinpoint.update();
        int xvalue = pinpoint.getEncoderY();
        while (xvalue - pinpoint.getEncoderY() <= tic) {
            pinpoint.update();
            backLeft.setPower(Speed);
            frontLeft.setPower(-Speed);
            backRight.setPower(-Speed);
            frontRight.setPower(Speed);
        }
        backLeft.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
    }
    public void strafeRight(double Speed, int time) {
        backLeft.setPower(-Speed);
        frontLeft.setPower(Speed);
        backRight.setPower(Speed);
        frontRight.setPower(-Speed);
        sleep(time);
        backLeft.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
    }
    public void strafeRightTics(double Speed, int tic) {
        pinpoint.update();
        int xvalue = pinpoint.getEncoderY();
        while (xvalue - pinpoint.getEncoderY() <= tic) {
            pinpoint.update();
            backLeft.setPower(-Speed);
            frontLeft.setPower(Speed);
            backRight.setPower(Speed);
            frontRight.setPower(-Speed);
        }
        backLeft.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
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
    public void turnRightTics(double Speed, int tic) {
        pinpoint.update();
        int xvalue = pinpoint.getEncoderX();
        while (xvalue - pinpoint.getEncoderX() <= tic) {
            pinpoint.update();
            backLeft.setPower(Speed);
            frontLeft.setPower(Speed);
            backRight.setPower(-Speed);
            frontRight.setPower(-Speed);
        }
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
    public void turnLeftTics(double Speed, int tic) {
        pinpoint.update();
        int xvalue = pinpoint.getEncoderX();
        while (xvalue - pinpoint.getEncoderX() <= tic) {
            pinpoint.update();
            backLeft.setPower(-Speed);
            frontLeft.setPower(-Speed);
            backRight.setPower(Speed);
            frontRight.setPower(Speed);
        }
        backLeft.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
    }

    public void moveBackward(double Speed, int time) {
        backLeft.setPower(Speed);
        frontLeft.setPower(Speed);
        backRight.setPower(Speed);
        frontRight.setPower(Speed);
        sleep(time);
        backLeft.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
    }
    public void moveBackwardTics(double Speed, int tic) {
        pinpoint.update();
        int xvalue = pinpoint.getEncoderX();
        while (xvalue - pinpoint.getEncoderX() <= tic) {
            backLeft.setPower(Speed);
            frontLeft.setPower(Speed);
            backRight.setPower(Speed);
            frontRight.setPower(Speed);
            pinpoint.update();
        }
        backLeft.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
    }
    public void moveForward(double Speed, int time) {
        backLeft.setPower(-Speed);
        frontLeft.setPower(-Speed);
        backRight.setPower(-Speed);
        frontRight.setPower(-Speed);
        sleep(time);
        backLeft.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
    }
    public void moveForwardTics(double Speed, int tic) {
        pinpoint.update();
        int xvalue = pinpoint.getEncoderX();
        while (xvalue - pinpoint.getEncoderX() <= tic) {
            backLeft.setPower(-Speed);
            frontLeft.setPower(-Speed);
            backRight.setPower(-Speed);
            frontRight.setPower(-Speed);
            pinpoint.update();
        }
        backLeft.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
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
                    backLeft.setPower(-localPower);
                    frontLeft.setPower(-localPower);
                    backRight.setPower(localPower);
                    frontRight.setPower(localPower);
                    sleep(sleepTime);
                    backLeft.setPower(0);
                    frontLeft.setPower(0);
                    backRight.setPower(0);
                    frontRight.setPower(0);
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
                } else {
                    break;
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