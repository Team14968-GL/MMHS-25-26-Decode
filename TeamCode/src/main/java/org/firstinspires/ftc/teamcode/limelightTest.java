package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@Autonomous(name = "limeLightTest")
public class limelightTest extends LinearOpMode {

    Limelight3A limelight;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        limelight.pipelineSwitch(0);

        int txMax = 0;
        int txMin = 0;
        int tyMax = 0;
        int tyMin = 0;
        int taMax = 0;
        int taMin = 0;

        waitForStart();
        while (opModeIsActive()) {

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
            } else {
                telemetry.addData("Limelight", "No Targets");

            }


            if (tx < txMin) {
                backLeft.setPower(.1);
                frontLeft.setPower(.1);
                backRight.setPower(-.1);
                frontLeft.setPower(-.1);
                sleep(50);
                backLeft.setPower(0);
                frontLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
            } else if (tx > txMax) {
                backLeft.setPower(-.1);
                frontLeft.setPower(-.1);
                backRight.setPower(.1);
                frontLeft.setPower(.1);
                sleep(50);
                backLeft.setPower(0);
                frontLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
            } else if (ta > taMax) {
                backLeft.setPower(-.1);
                frontLeft.setPower(-.1);
                backRight.setPower(-.1);
                frontLeft.setPower(-.1);
                sleep(50);
                backLeft.setPower(0);
                frontLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
            } else if (ta > taMax) {
                backLeft.setPower(.1);
                frontLeft.setPower(.1);
                backRight.setPower(.1);
                frontLeft.setPower(.1);
                sleep(50);
                backLeft.setPower(0);
                frontLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
            }
        }
    }
}


