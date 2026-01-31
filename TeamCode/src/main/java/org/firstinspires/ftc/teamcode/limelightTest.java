package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
@Disabled
@Autonomous(name = "limeLightTest")
public class limelightTest extends LinearOpMode {

    Limelight3A limelight;
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


        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        limelight.pipelineSwitch(0);



        waitForStart();



        while (opModeIsActive()) {
            telemetry.update();
            localize();

        }
    }
    public void localize() {
        //boolean localizing = true;

            LLResult result = limelight.getLatestResult();
            double tx = 0;
            double ty;
            double ta = 0;
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
                    backLeft.setPower(-power);
                    frontLeft.setPower(-power);
                    backRight.setPower(power);
                    frontRight.setPower(power);
                    sleep(sleepTime);
                    backLeft.setPower(0);
                    frontLeft.setPower(0);
                    backRight.setPower(0);
                    frontRight.setPower(0);
                } else if (tx > txMax) {
                    //turn left
                    backLeft.setPower(power);
                    frontLeft.setPower(power);
                    backRight.setPower(-power);
                    frontRight.setPower(-power);
                    sleep(sleepTime);
                    backLeft.setPower(0);
                    frontLeft.setPower(0);
                    backRight.setPower(0);
                    frontRight.setPower(0);
                } else if (ta > taMax) {
                    //move backward
                    backLeft.setPower(power);
                    frontLeft.setPower(power);
                    backRight.setPower(power);
                    frontRight.setPower(power);
                    sleep(sleepTime);
                    backLeft.setPower(0);
                    frontLeft.setPower(0);
                    backRight.setPower(0);
                    frontRight.setPower(0);
                } else if (ta < taMin) {
                    //move Forward
                    backLeft.setPower(-power);
                    frontLeft.setPower(-power);
                    backRight.setPower(-power);
                    frontRight.setPower(-power);
                    sleep(sleepTime);
                    backLeft.setPower(0);
                    frontLeft.setPower(0);
                    backRight.setPower(0);
                    frontRight.setPower(0);
                } else {
                    telemetry.addData("done", 0);
                }

            } else {
                telemetry.addData("Limelight", "No Targets");

                backLeft.setPower(0);
                frontLeft.setPower(0);
                backRight.setPower(0);
                frontRight.setPower(0);
            }
    }
}