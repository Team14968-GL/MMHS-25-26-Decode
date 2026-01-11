package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name = "Launch Power Finder LE")
public class LaunchPwrFinder extends LinearOpMode {

    /**
     * Describe this function...
     */
    @Override
    public void runOpMode() {
        boolean launchOn;
        double pwr = 0;

        DcMotor leftLauncher = hardwareMap.get(DcMotor.class, "leftLauncher");
        DcMotor rightLauncher = hardwareMap.get(DcMotor.class, "rightLauncher");
        Servo scoop = hardwareMap.get(Servo.class, "scoop");

        leftLauncher.setDirection(DcMotor.Direction.REVERSE);
        rightLauncher.setDirection(DcMotor.Direction.FORWARD);
        launchOn = false;
        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                if (gamepad1.dpad_up) {
                    pwr += 0.05;
                    sleep(250);
                } else if (gamepad1.dpad_down) {
                    pwr -= 0.05;
                    sleep(250);
                }
                if (gamepad1.circle) {
                    scoop.setPosition(0);
                    sleep(1000);
                    scoop.setPosition(0.5);
                    sleep(1000);
                    scoop.setPosition(0);
                }
                if (gamepad1.cross && launchOn) {
                    launchOn = false;
                    sleep(500);
                } else if (gamepad1.cross && !launchOn) {
                    launchOn = true;
                    sleep(500);
                }
                if (launchOn) {
                    leftLauncher.setPower(pwr);
                    rightLauncher.setPower(pwr);
                } else {
                    leftLauncher.setPower(0);
                    rightLauncher.setPower(0);
                }
                // Put loop blocks here.
                telemetry.update();
                telemetry.addData("Launcher Power", pwr * 100 + "%");
                telemetry.addData("Raw", pwr);
            }
        }
    }
}