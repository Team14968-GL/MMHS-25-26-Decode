package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

final public class RightFrontMotorTest {
    @TeleOp
    public class RFSingleMotorTest extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException {
            DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");

            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            waitForStart();

            while (opModeIsActive()) {
                // Hold A to power right-front forward at 0.5
                double power = gamepad1.a ? 0.5 : 0.0;
                frontRight.setPower(power);

                telemetry.addData("RF power", power);
                telemetry.addData("RF ticks", frontRight.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}
