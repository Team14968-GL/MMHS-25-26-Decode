package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Disabled
@Autonomous(name = "movementTest")
public class movementTest extends LinearOpMode {
    public  DcMotor backLeft;
    public  DcMotor backRight;
    public  DcMotor frontLeft;
    public DcMotor frontRight;
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pinpoint.initialize();
        double ticPerIn = 25.47;
        waitForStart();
        if (opModeIsActive()){
            turnRightTics(.5, 90);
            sleep(1000);
            turnLeftTics(.5, 90);
            sleep(1000);
            moveBackwardTics(.3, 5000);
            sleep(1000);
            moveForwardTics(.3, 5000);
            sleep(1000);
            strafeLeftTics(.3, 5000);
            sleep(1000);
            strafeRightTics(.3, 5000);
        }

    }
    public void strafeLeftTics(double Speed, int tic) {
        pinpoint.update();
        int yValue = pinpoint.getEncoderY();
        while (yValue - pinpoint.getEncoderY() <= tic) {
            pinpoint.update();
            backLeft.setPower(Speed);
            frontLeft.setPower(-Speed);
            backRight.setPower(-Speed);
            frontRight.setPower(Speed);
            telemetry.addData("yEncoder", pinpoint.getEncoderY());
            telemetry.addData("yValue", yValue);
            telemetry.addData("y", pinpoint.getEncoderY()-yValue);
            telemetry.update();
        }
        backLeft.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
    }
    public void strafeRightTics(double Speed, int tic) {
        pinpoint.update();
        int yValue = pinpoint.getEncoderY();
        while (pinpoint.getEncoderY() - yValue <= tic) {
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
    public void turnRightTics(double Speed, double deg) {
        pinpoint.update();
        double degValue = pinpoint.getHeading(AngleUnit.DEGREES);
        while (degValue - pinpoint.getHeading(AngleUnit.DEGREES) <= deg) {
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
    public void turnLeftTics(double Speed, double deg) {
        pinpoint.update();
        double degValue = pinpoint.getHeading(AngleUnit.DEGREES);
        while (pinpoint.getHeading(AngleUnit.DEGREES) - degValue <= deg) {
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
    public void moveBackwardTics(double Speed, int tic) {
        pinpoint.update();
        int xValue = pinpoint.getEncoderX(); // \/
        while (pinpoint.getEncoderX() - xValue <= tic) {
            pinpoint.update();
            backLeft.setPower(Speed);
            frontLeft.setPower(Speed);
            backRight.setPower(Speed);
            frontRight.setPower(Speed);
            telemetry.addData("xEncoder", pinpoint.getEncoderX());
            telemetry.addData("xValue", xValue);
            telemetry.update();
        }
        backLeft.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
    }
    public void moveForwardTics(double Speed, int tic) {
        pinpoint.update();
        int xValue = pinpoint.getEncoderX();
        while (xValue - pinpoint.getEncoderX() <= tic) {
            pinpoint.update();
            backLeft.setPower(-Speed);
            frontLeft.setPower(-Speed);
            backRight.setPower(-Speed);
            frontRight.setPower(-Speed);
        }
        backLeft.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
    }

}