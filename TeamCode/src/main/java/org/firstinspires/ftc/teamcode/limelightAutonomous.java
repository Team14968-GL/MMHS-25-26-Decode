package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.ArrayList;

import java.util.List;

@Autonomous(name = "limeLightAutonomous")
public class limelightAutonomous extends LinearOpMode {

    Limelight3A limelight;

    public int id;
    public ArrayList<Integer> IDs = new ArrayList<>();
    boolean telem = false;
    public  DcMotor backLeft;
    public  DcMotor backRight;
    public  DcMotor frontLeft;
    public DcMotor frontRight;

    double txMax = 15;
    double txMin = 9;
    double tyMax = 13.5;
    double tyMin = 12;
    double taMax = 2.35;
    double taMin = 2.07;

    double power = .5;
    double localPower = .3;

    int sleepTime = 50;
    boolean processTrig = true;

    double xValue = 0;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        limelight.pipelineSwitch(0);

        LLResultTypes.FiducialResult fiducialResult = null;

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
        }
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

            if (!fiducialList.isEmpty()) {
                telemetry.addData("Detections Found", fiducialList.size());

                // Iterate through each detected tag
                for (LLResultTypes.FiducialResult fiducial : fiducialList) {
                    id = fiducial.getFiducialId();
                    IDs.add(id);
                    telemetry.addData("Tag ID", id);
                }
                processTrig = false;
            } else {
                telemetry.addData("Detections Found", "None");
            }
        } else {
            telemetry.addData("Limelight Data", "Invalid or Stale");
            assert result != null;
            telemetry.addData("Staleness", result.getStaleness());
        }
        telemetry.update();

        return id;
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