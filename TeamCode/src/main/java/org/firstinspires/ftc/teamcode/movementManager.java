package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class movementManager extends LinearOpMode {
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor rightFront;

    public void setup(){
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
    }

    public void strafeLeft ( double Speed, int time){
        setup();

        leftBack.setPower(Speed);
        leftFront.setPower(-Speed);
        rightBack.setPower(-Speed);
        rightFront.setPower(Speed);

        sleep(time);

        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    public void strafeRight ( double Speed, int time){
        setup();

        leftBack.setPower(-Speed);
        leftFront.setPower(Speed);
        rightBack.setPower(Speed);
        rightFront.setPower(-Speed);

        sleep(time);

        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    public void turnRight ( double Speed, int time){
        setup();

        leftBack.setPower(Speed);
        leftFront.setPower(Speed);
        rightBack.setPower(-Speed);
        rightFront.setPower(-Speed);

        sleep(time);

        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    public void turnLeft ( double Speed, int time){
        setup();

        leftBack.setPower(-Speed);
        leftFront.setPower(-Speed);
        rightBack.setPower(Speed);
        rightFront.setPower(Speed);

        sleep(time);

        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    public void moveBackward ( double Speed, int time){
        setup();

        leftBack.setPower(Speed);
        leftFront.setPower(Speed);
        rightBack.setPower(Speed);
        rightFront.setPower(Speed);

        sleep(time);

        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    public void moveForward ( double Speed, int time){
        setup();

        leftBack.setPower(-Speed);
        leftFront.setPower(-Speed);
        rightBack.setPower(-Speed);
        rightFront.setPower(-Speed);

        sleep(time);

        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }
}