package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "Basic Autonomous RJ LE", preselectTeleOp = "BasicTeleopRJ")
public class BasicAutoRJ extends LinearOpMode {

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue Comment
     * Blocks show where to place Initialization code (runs once, after touching the DS INIT
     * button, and before touching the DS Start arrow), Run code (runs once, after touching
     * Start), and Loop code (runs repeatedly while the OpMode is active, namely not Stopped).
     */
    @Override
    public void runOpMode() {
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        // Put initialization blocks here.
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            backLeft.setPower(0.7);
            backRight.setPower(-0.7);
            frontRight.setPower(0.7);
            frontLeft.setPower(-0.7);
            sleep(1000);
            backLeft.setPower(0);
            backRight.setPower(0);
            frontRight.setPower(0);
            frontLeft.setPower(0);
        }
    }
}