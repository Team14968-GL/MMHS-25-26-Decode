package org.firstinspires.ftc.teamcode;


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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "ForwardLeaveAuto")
public class ForwardLeaveAuto extends LinearOpMode {
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
        double ticPerIn = 254.7;
        waitForStart();
        if (opModeIsActive()){
            moveBackwardTics(.3, ticPerIn*30);
        }

    }
    public void strafeLeftTics(double Speed, int tic) {
        pinpoint.update();
        int yvalue = pinpoint.getEncoderY();
        while (yvalue - pinpoint.getEncoderY() <= tic) {
            pinpoint.update();
            backLeft.setPower(Speed);
            frontLeft.setPower(-Speed);
            backRight.setPower(-Speed);
            frontRight.setPower(Speed);
            telemetry.addData("yencoder", pinpoint.getEncoderY());
            telemetry.addData("yvalue", yvalue);
            telemetry.addData("y", pinpoint.getEncoderY()-yvalue);
            telemetry.update();
        }
        backLeft.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
    }
    public void strafeRightTics(double Speed, int tic) {
        pinpoint.update();
        int yvalue = pinpoint.getEncoderY();
        while (pinpoint.getEncoderY() - yvalue <= tic) {
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
        double degvalue = pinpoint.getHeading(AngleUnit.DEGREES);
        while (degvalue - pinpoint.getHeading(AngleUnit.DEGREES) <= deg) {
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
        double degvalue = pinpoint.getHeading(AngleUnit.DEGREES);
        while (pinpoint.getHeading(AngleUnit.DEGREES) - degvalue <= deg) {
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
    public void moveBackwardTics(double Speed, double tic) {
        pinpoint.update();
        int xvalue = pinpoint.getEncoderX(); // \/
        while (pinpoint.getEncoderX() - xvalue <= tic) {
            pinpoint.update();
            backLeft.setPower(Speed);
            frontLeft.setPower(Speed);
            backRight.setPower(Speed);
            frontRight.setPower(Speed);
            telemetry.addData("xencoder", pinpoint.getEncoderX());
            telemetry.addData("xvalue", xvalue);
            telemetry.update();
        }
        backLeft.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
    }
    public void moveForwardTics(double Speed, int tic) {
        pinpoint.update();
        int xvalue = pinpoint.getEncoderX();
        while (xvalue - pinpoint.getEncoderX() <= tic) {
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