package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;

public final class MMHS26lib {
    public static class motion {
        public static void strafeLeft(double Speed, int time) throws InterruptedException {
            final DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
            final DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");
            final DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
            final DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");

            leftBack.setDirection(DcMotor.Direction.FORWARD);
            rightBack.setDirection(DcMotor.Direction.FORWARD);
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);

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
        public static void strafeRight(double Speed, int time) throws InterruptedException {
            final DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
            final DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");
            final DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
            final DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");

            leftBack.setDirection(DcMotor.Direction.FORWARD);
            rightBack.setDirection(DcMotor.Direction.FORWARD);
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);

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
        public static void turnRight(double Speed, int time) throws InterruptedException {
            final DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
            final DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");
            final DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
            final DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");

            leftBack.setDirection(DcMotor.Direction.FORWARD);
            rightBack.setDirection(DcMotor.Direction.FORWARD);
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);

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
        public static void turnLeft(double Speed, int time) throws InterruptedException {
            final DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
            final DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");
            final DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
            final DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");

            leftBack.setDirection(DcMotor.Direction.FORWARD);
            rightBack.setDirection(DcMotor.Direction.FORWARD);
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);

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
        public static void moveBackward(double Speed, int time) throws InterruptedException {
            final DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
            final DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");
            final DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
            final DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");

            leftBack.setDirection(DcMotor.Direction.FORWARD);
            rightBack.setDirection(DcMotor.Direction.FORWARD);
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);

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
        public static void moveForward(double Speed, int time) throws InterruptedException {
            final DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
            final DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");
            final DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
            final DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");

            leftBack.setDirection(DcMotor.Direction.FORWARD);
            rightBack.setDirection(DcMotor.Direction.FORWARD);
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);

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
        public static void halt() throws InterruptedException {
            final DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
            final DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");
            final DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
            final DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");

            leftBack.setDirection(DcMotor.Direction.FORWARD);
            rightBack.setDirection(DcMotor.Direction.FORWARD);
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.FORWARD);

            leftBack.setPower(0);
            leftFront.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);
        }
    }
    public static class limelight {
        static Limelight3A limelight;
        final GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver .class, "pinpoint");

        public static void localizer(double localPower, int sleepTime) throws InterruptedException {
            //boolean localizing = true;
            while (true) {
                LLResult result = limelight.getLatestResult();
                double txMax = 14.700;
                double txMin = 14.400;
                double tyMax = 13.5;
                double tyMin = 12;
                double taMax = .88;
                double taMin = .91;
                double tx;
                double ty;
                double ta;
                if (result != null && result.isValid()) {
                    tx = result.getTx();
                    ty = result.getTy(); // How far up or down the target is (degrees)
                    ta = result.getTa(); // How big the target looks (0%-100% of the image)

                    telemetry.addData("Target X", tx);
                    telemetry.addData("Target Y", ty);
                    telemetry.addData("Target Area", ta);
                    telemetry.update();

                    if (tx < txMin) {
                        // turn right
                        motion.turnRight(localPower, sleepTime);
                    } else if (tx > txMax) {
                        //turn left
                        motion.turnLeft(localPower, sleepTime);
                    } else if (ta > taMax) {
                        //move backward
                        motion.moveBackward(localPower, sleepTime);
                    } else if (ta < taMin) {
                        //move Forward
                        motion.moveForward(localPower, sleepTime);
                    } else {
                        break;
                    }

                } else {
                    telemetry.addData("Limelight", "No Targets");
                    telemetry.update();

                    motion.halt();
                }
            }
        }
    }
    public static class roadRunner {
        public static void splineTo(double x, double y, double tangent, Pose2d startingPose){
            MecanumDrive drive = new MecanumDrive(hardwareMap, startingPose);
            TrajectoryActionBuilder splineTo = drive.actionBuilder(startingPose)
                    .splineTo(new Vector2d(x, y), Math.toRadians(tangent));
            Actions.runBlocking(
                    new SequentialAction(
                            splineTo.build()));
        }
    }
}
