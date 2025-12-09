package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Voltage Drive Test") // use @Autonomous for autonomous and use @TeleOp for teleop
public class VelocityDriveTest extends LinearOpMode {

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue Comment
     * Blocks show where to place Initialization code (runs once, after touching the DS INIT
     * button, and before touching the DS Start arrow), Run code (runs once, after touching
     * Start), and Loop code (runs repeatedly while the OpMode is active, namely not Stopped).
     */
    @Override
    public void runOpMode() {
        float controlY;
        float controlX;
        float controlRX;
        float speed;
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "rightBack");
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        speed = 50;

        // Put initialization blocks here.
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                controlY = gamepad1.left_stick_x;
                controlX = gamepad1.right_stick_x;
                controlRX = gamepad1.right_stick_y;
                frontLeft.setVelocity(((controlY - controlX) + controlRX) * speed);
                backLeft.setVelocity(((controlY + controlX) + controlRX) * speed);
                frontRight.setVelocity(((controlY - controlX) - controlRX) * speed);
                backRight.setVelocity(((controlY + controlX) - controlRX) * speed);
            }
        }
    }
}