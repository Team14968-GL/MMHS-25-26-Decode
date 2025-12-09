package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Voltage Drive Test")
public class VelocityDriveTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        float controlY;
        float controlX;
        float controlRX;
        float rpm;
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "rightBack");
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");

        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()) {
            rpm = 1;
            while (opModeIsActive()) {
                controlY = gamepad1.left_stick_x;
                controlX = gamepad1.right_stick_x;
                controlRX = gamepad1.right_stick_y;
                frontLeft.setVelocity(((controlY - controlX) + controlRX) * rpm / 60);
                backLeft.setVelocity(((controlY + controlX) + controlRX) * rpm / 60);
                frontRight.setVelocity(((controlY - controlX) - controlRX) * rpm / 60);
                backRight.setVelocity(((controlY + controlX) - controlRX) * rpm / 60);
            }
        }
    }
}