package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Launch Autonomous RJ LE", preselectTeleOp = "BasicTeleopRJ")
public class LaunchAutoRJ extends LinearOpMode {

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue Comment
     * Blocks show where to place Initialization code (runs once, after touching the DS INIT
     * button, and before touching the DS Start arrow), Run code (runs once, after touching
     * Start), and Loop code (runs repeatedly while the OpMode is active, namely not Stopped).
     */
    @Override
    public void runOpMode() {
        double Wheel_PWR;

        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        Servo backDoor = hardwareMap.get(Servo.class, "backDoor");
        Servo scoop = hardwareMap.get(Servo.class, "scoop");
        DcMotor leftLauncher = hardwareMap.get(DcMotor.class, "leftLauncher");
        DcMotor rightLauncher = hardwareMap.get(DcMotor.class, "rightLauncher");

        // Put initialization blocks here.
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backDoor.setPosition(0);
        scoop.setPosition(0);
        Wheel_PWR = 0.55;
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            // Put loop blocks here.
            backLeft.setPower(Wheel_PWR);
            backRight.setPower(Wheel_PWR);
            frontLeft.setPower(Wheel_PWR);
            frontRight.setPower(Wheel_PWR);
            sleep(750);
            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
            sleep(500);
            leftLauncher.setPower(-0.45);
            rightLauncher.setPower(0.45);
            sleep(500);
            scoop.setPosition(0.5);
            sleep(2000);
            leftLauncher.setPower(0);
            rightLauncher.setPower(0);
            backLeft.setPower(-Wheel_PWR);
            backRight.setPower(Wheel_PWR);
            frontLeft.setPower(Wheel_PWR);
            frontRight.setPower(-Wheel_PWR);
            sleep(1250);
            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
            telemetry.update();
        }
    }
}