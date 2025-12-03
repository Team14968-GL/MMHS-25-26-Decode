package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name = "Voltage Curve Tester LE")
public class VoltageCurveTester extends LinearOpMode {

    /**
     * Tool that allows for autonomous collection of voltage and velocity data
     */
    @Override
    public void runOpMode() {
        VoltageSensor controlHub_VoltageSensor;
        controlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor leftLauncher = hardwareMap.get(DcMotor.class, "leftLauncher");
        DcMotor rightLauncher = hardwareMap.get(DcMotor.class, "rightLauncher");

        waitForStart();
        if (opModeIsActive()) {
            // WARNING: This OpMode is data intensive and will likely create a VERY large log file containing voltage and velocity data for plotting onto a graph.
            telemetry.update();
            telemetry.addData("WARNING", "This OpMode is data intensive and will likely create a VERY large log file containing voltage and velocity data for plotting onto a graph");
            telemetry.update();
            while (opModeIsActive()) {
                telemetry.addData("CHVoltage", controlHub_VoltageSensor.getVoltage());
                RobotLog.ii("DbgLog", "cHVol: " + controlHub_VoltageSensor.getVoltage());
                telemetry.addData("fLTicks/s-Vel", ((DcMotorEx) frontLeft).getVelocity());
                RobotLog.ii("DbgLog", "fLVel: " + ((DcMotorEx) frontLeft).getVelocity());
                telemetry.addData("fR-Ticks/s-Vel", ((DcMotorEx) frontRight).getVelocity());
                RobotLog.ii("DbgLog", "fRVel: " + ((DcMotorEx) frontRight).getVelocity());
                telemetry.addData("bL-Ticks/s-Vel", ((DcMotorEx) backLeft).getVelocity());
                RobotLog.ii("DbgLog", "bLVel: " + ((DcMotorEx) backLeft).getVelocity());
                telemetry.addData("bR-Ticks/s-Velocity", ((DcMotorEx) backRight).getVelocity());
                RobotLog.ii("DbgLog", "bRVel: " + ((DcMotorEx) backRight).getVelocity());
                telemetry.addData("lL-Ticks/s-Velocity", ((DcMotorEx) leftLauncher).getVelocity());
                RobotLog.ii("DbgLog", "lLVel: " + ((DcMotorEx) leftLauncher).getVelocity());
                telemetry.addData("rL-Ticks/s-Velocity", ((DcMotorEx) rightLauncher).getVelocity());
                RobotLog.ii("DbgLog", "rLVel: " + ((DcMotorEx) rightLauncher).getVelocity());
                telemetry.update();
            }
        }
    }
}