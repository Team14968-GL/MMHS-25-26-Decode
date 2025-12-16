package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LaunchFunc {

    private DcMotor leftLauncher;
    private DcMotor rightLauncher;
    private Servo backDoor;
    private Servo scoop;
    private Servo goofyAhhhhFrontDoor;

    public static int triangleFuncRunning;
    public static double launcherSpeed;
    private static ElapsedTime triangleClock;

    private HardwareMap hardwareMap = null;

    public LaunchFunc(HardwareMap hwMap) {
        this.hardwareMap = hwMap;
    }

    public void initHardware() {
        leftLauncher = hardwareMap.get(DcMotor.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotor.class, "rightLauncher");
        backDoor = hardwareMap.get(Servo.class, "backDoor");
        scoop = hardwareMap.get(Servo.class, "scoop");
        goofyAhhhhFrontDoor = hardwareMap.get(Servo.class, "goofyAhhhhFrontDoor");

        leftLauncher.setDirection(DcMotor.Direction.REVERSE);
    }


    private void launchMotorOnTriangle() {
        ((DcMotorEx) leftLauncher).setVelocity(launcherSpeed * Math.abs(triangleFuncRunning - 1));
        ((DcMotorEx) rightLauncher).setVelocity(launcherSpeed * Math.abs(triangleFuncRunning - 1));
    }

    public void launch() throws InterruptedException {
        triangleFuncRunning = 0;
        launchMotorOnTriangle();
        backDoor.setPosition(0);
        Thread.sleep(1000);
        leftLauncher.setPower(0);
        rightLauncher.setPower(0);
        triangleFuncRunning = 0;
    }
}
