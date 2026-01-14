package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class MMHS26Lib {
    private static DcMotor leftBack;
    private static DcMotor rightBack;
    private static DcMotor leftFront;
    private static DcMotor rightFront;
    static HardwareMap hwMap;
    private static GoBildaPinpointDriver pinpoint;
    private static CRServo LED1;
    private static ArrayList<CRServo> leds;
    public static Limelight3A limelight;


    public MMHS26Lib(HardwareMap hardwareMap){
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.initialize();

        LED1 = hardwareMap.get(CRServo.class, "Led1");
        leds = new ArrayList<>(Arrays.asList(null, LED1));

        hwMap = hardwareMap;
    }


    @Config
    public static class debug {
        private static boolean debugTelemetry = false;
        public static void debugTelemetry(boolean enabled){
            debugTelemetry = enabled;
        }
    }
    private static void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public static Pose2d currentPose(){
        return(new Pose2d( new Vector2d(pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH)), pinpoint.getHeading(AngleUnit.RADIANS)));
    }

    public static class motion {

        public motion() {
            super();
        }

        public static void strafeLeft(double Speed, long time){
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
        public static void strafeRight(double Speed, long time){
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
        public static void turnRight(double Speed, long time){
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
        public static void turnLeft(double Speed, long time){
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
        public static void moveBackward(double Speed, long time){
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
        public static void moveForward(double Speed, long time) {
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
        public static void halt(){
            leftBack.setPower(0);
            leftFront.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);
        }
    }
    public static class limelight {
        public limelight(){
            super();
        }

        public static void localizer(double localPower, int sleepTime){
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
                    if(debug.debugTelemetry){
                        telemetry.addData("Target X", tx);
                        telemetry.addData("Target Y", ty);
                        telemetry.addData("Target Area", ta);
                        telemetry.update();
                    }

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
                    if(debug.debugTelemetry) {
                        telemetry.addData("Limelight", "No Targets");
                        telemetry.update();
                    }
                    motion.halt();
                }
            }
        }
        static int processLimeLightResults() {
            int id = 0;
            ArrayList<Integer> IDs = new ArrayList<>();
            double tx;
            double ty;
            double ta;

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                // Get the list of ALL detected fiducials (AprilTags)
                List<LLResultTypes.FiducialResult> fiducialList = result.getFiducialResults();

                tx = result.getTx();
                ty = result.getTy(); // How far up or down the target is (degrees)
                ta = result.getTa(); // How big the target looks (0%-100% of the image)

                if(debug.debugTelemetry) {
                    telemetry.addData("Target X", tx);
                    telemetry.addData("Target Y", ty);
                    telemetry.addData("Target Area", ta);
                    telemetry.update();
                }

                if (!fiducialList.isEmpty()) {
                    if(debug.debugTelemetry) {
                        telemetry.addData("Detections Found", fiducialList.size());
                        telemetry.update();
                    }
                    // Iterate through each detected tag
                    for (LLResultTypes.FiducialResult fiducial : fiducialList) {
                        id = fiducial.getFiducialId();
                        IDs.add(id);
                        if(debug.debugTelemetry) {
                            telemetry.addData("Tag ID", id);
                            telemetry.update();
                        }
                    }
                } else {
                    if(debug.debugTelemetry) {
                        telemetry.addData("Detections Found", "None");
                        telemetry.update();
                    }
                }
            } else {
                if(debug.debugTelemetry) {
                    telemetry.addData("Limelight Data", "Invalid or Stale");
                    assert result != null;
                    telemetry.addData("Staleness", result.getStaleness());
                    telemetry.update();
                }
            }
            if(debug.debugTelemetry) {
                telemetry.update();
            }
            return id;
        }
    }
    public static class roadRunner  {
        public roadRunner(){
            super();
        }
        public static class spline {
            public spline(){
                super();
            }

            public static Pose2d splineTo(double x, double y, double tangent, Pose2d startingPose) {

                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder splineTo = drive.actionBuilder(startingPose)
                        .splineTo(new Vector2d(x, y), Math.toRadians(tangent));
                Actions.runBlocking(
                        new SequentialAction(
                                splineTo.build()));

                return(currentPose());
            }
            public static Pose2d splineToConstantHeading(double x, double y, double tangent, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder splineToConstantHeading = drive.actionBuilder(startingPose)
                        .splineToConstantHeading(new Vector2d(x, y), Math.toRadians(tangent));
                Actions.runBlocking(
                        new SequentialAction(
                                splineToConstantHeading.build()));


                return(currentPose());
            }
            public static Pose2d splineToLinearHeading(double x, double y, double angle, double tangent, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder splineToLinearHeading = drive.actionBuilder(startingPose)
                        .splineToLinearHeading(new Pose2d(new Vector2d(x, y), angle), tangent);
                Actions.runBlocking(
                        new SequentialAction(
                                splineToLinearHeading.build()));


                return(currentPose());
            }
            public static Pose2d splineToSplineHeading(double x, double y, double angle, double tangent, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder splineToSplineHeading = drive.actionBuilder(startingPose)
                        .splineToSplineHeading(new Pose2d(new Vector2d(x, y), angle), tangent);
                Actions.runBlocking(
                        new SequentialAction(
                                splineToSplineHeading.build()));

                return(currentPose());
            }
        }
        public static class strafe {
            public strafe(){
                super();
            }
            public static Pose2d strafeTo(double x, double y, boolean VelCon, boolean AccCon, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder strafeTo;
                if (VelCon && AccCon) {
                    strafeTo = drive.actionBuilder(startingPose)
                            .strafeTo(new Vector2d(x, y), drive.defaultVelConstraint, drive.defaultAccelConstraint);
                } else if (VelCon) {
                    strafeTo = drive.actionBuilder(startingPose)
                            .strafeTo(new Vector2d(x, y), drive.defaultVelConstraint, null);
                } else if (AccCon) {
                    strafeTo = drive.actionBuilder(startingPose)
                            .strafeTo(new Vector2d(x, y), null, drive.defaultAccelConstraint);
                } else {
                    strafeTo = drive.actionBuilder(startingPose)
                            .strafeTo(new Vector2d(x, y));
                }

                Actions.runBlocking(
                        new SequentialAction(
                                strafeTo.build()));

                return(currentPose());
            }
            public static Pose2d strafeToConstantHeading(double x, double y, boolean VelCon, boolean AccCon, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder strafeToConstantHeading;
                if (VelCon && AccCon) {
                    strafeToConstantHeading = drive.actionBuilder(startingPose)
                            .strafeToConstantHeading(new Vector2d(x, y), drive.defaultVelConstraint, drive.defaultAccelConstraint);
                } else if (VelCon) {
                    strafeToConstantHeading = drive.actionBuilder(startingPose)
                            .strafeToConstantHeading(new Vector2d(x, y), drive.defaultVelConstraint, null);
                } else if (AccCon) {
                    strafeToConstantHeading = drive.actionBuilder(startingPose)
                            .strafeToConstantHeading(new Vector2d(x, y), null, drive.defaultAccelConstraint);
                } else {
                    strafeToConstantHeading = drive.actionBuilder(startingPose)
                            .strafeToConstantHeading(new Vector2d(x, y));
                }

                Actions.runBlocking(
                        new SequentialAction(
                                strafeToConstantHeading.build()));

                return(currentPose());
            }
            public static Pose2d strafeToLinearHeading(double x, double y, double angle, boolean VelCon, boolean AccCon, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder strafeToLinearHeading;
                if (VelCon && AccCon) {
                    strafeToLinearHeading = drive.actionBuilder(startingPose)
                            .strafeToLinearHeading(new Vector2d(x, y), Math.toRadians(angle), drive.defaultVelConstraint, drive.defaultAccelConstraint);
                } else if (VelCon) {
                    strafeToLinearHeading = drive.actionBuilder(startingPose)
                            .strafeToLinearHeading(new Vector2d(x, y), Math.toRadians(angle), drive.defaultVelConstraint, null);
                } else if (AccCon) {
                    strafeToLinearHeading = drive.actionBuilder(startingPose)
                            .strafeToLinearHeading(new Vector2d(x, y), Math.toRadians(angle), null, drive.defaultAccelConstraint);
                } else {
                    strafeToLinearHeading = drive.actionBuilder(startingPose)
                            .strafeToLinearHeading(new Vector2d(x, y), Math.toRadians(angle));
                }

                Actions.runBlocking(
                        new SequentialAction(
                                strafeToLinearHeading.build()));

                return(currentPose());
            }
            public static Pose2d strafeToSplineHeading(double x, double y, double angle, boolean VelCon, boolean AccCon, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder strafeToSplineHeading;
                if (VelCon && AccCon) {
                    strafeToSplineHeading = drive.actionBuilder(startingPose)
                            .strafeToSplineHeading(new Vector2d(x, y), Math.toRadians(angle), drive.defaultVelConstraint, drive.defaultAccelConstraint);
                } else if (VelCon) {
                    strafeToSplineHeading = drive.actionBuilder(startingPose)
                            .strafeToSplineHeading(new Vector2d(x, y), Math.toRadians(angle), drive.defaultVelConstraint, null);
                } else if (AccCon) {
                    strafeToSplineHeading = drive.actionBuilder(startingPose)
                            .strafeToSplineHeading(new Vector2d(x, y), Math.toRadians(angle), null, drive.defaultAccelConstraint);
                } else {
                    strafeToSplineHeading = drive.actionBuilder(startingPose)
                            .strafeToSplineHeading(new Vector2d(x, y), Math.toRadians(angle));
                }

                Actions.runBlocking(
                        new SequentialAction(
                                strafeToSplineHeading.build()));

                return(currentPose());
            }
        }

        public static class lineTo {
            public lineTo(){
                super();
            }
            public static Pose2d lineToX(double X, boolean VelCon, boolean AccCon, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder lineToX;
                if (VelCon && AccCon) {
                    lineToX = drive.actionBuilder(startingPose)
                            .lineToX(X, drive.defaultVelConstraint, drive.defaultAccelConstraint);
                } else if (VelCon) {
                    lineToX = drive.actionBuilder(startingPose)
                            .lineToX(X, drive.defaultVelConstraint, null);
                } else if (AccCon) {
                    lineToX = drive.actionBuilder(startingPose)
                            .lineToX(X, null, drive.defaultAccelConstraint);
                } else {
                    lineToX = drive.actionBuilder(startingPose)
                            .lineToX(X);
                }

                Actions.runBlocking(
                        new SequentialAction(
                                lineToX.build()));

                return(currentPose());
            }
            public static Pose2d lineToXConstantHeading(double X, boolean VelCon, boolean AccCon, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder lineToXConstantHeading;
                if (VelCon && AccCon) {
                    lineToXConstantHeading = drive.actionBuilder(startingPose)
                            .lineToXConstantHeading(X, drive.defaultVelConstraint, drive.defaultAccelConstraint);
                } else if (VelCon) {
                    lineToXConstantHeading = drive.actionBuilder(startingPose)
                            .lineToXConstantHeading(X, drive.defaultVelConstraint, null);
                } else if (AccCon) {
                    lineToXConstantHeading = drive.actionBuilder(startingPose)
                            .lineToXConstantHeading(X, null, drive.defaultAccelConstraint);
                } else {
                    lineToXConstantHeading = drive.actionBuilder(startingPose)
                            .lineToXConstantHeading(X);
                }

                Actions.runBlocking(
                        new SequentialAction(
                                lineToXConstantHeading.build()));

                return(currentPose());
            }
            public static Pose2d lineToXLinearHeading(double X, double angle, boolean VelCon, boolean AccCon, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder lineToXLinearHeading;
                if (VelCon && AccCon) {
                    lineToXLinearHeading = drive.actionBuilder(startingPose)
                            .lineToXLinearHeading(X, Math.toRadians(angle), drive.defaultVelConstraint, drive.defaultAccelConstraint);
                } else if (VelCon) {
                    lineToXLinearHeading = drive.actionBuilder(startingPose)
                            .lineToXLinearHeading(X, Math.toRadians(angle), drive.defaultVelConstraint, null);
                } else if (AccCon) {
                    lineToXLinearHeading = drive.actionBuilder(startingPose)
                            .lineToXLinearHeading(X, Math.toRadians(angle), null, drive.defaultAccelConstraint);
                } else {
                    lineToXLinearHeading = drive.actionBuilder(startingPose)
                            .lineToXLinearHeading(X, Math.toRadians(angle));
                }

                Actions.runBlocking(
                        new SequentialAction(
                                lineToXLinearHeading.build()));

                return(currentPose());
            }
            public static Pose2d lineToXSplineHeading(double X, double angle, boolean VelCon, boolean AccCon, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder lineToXSplineHeading;
                if (VelCon && AccCon) {
                    lineToXSplineHeading = drive.actionBuilder(startingPose)
                            .lineToXSplineHeading(X, Math.toRadians(angle), drive.defaultVelConstraint, drive.defaultAccelConstraint);
                } else if (VelCon) {
                    lineToXSplineHeading = drive.actionBuilder(startingPose)
                            .lineToXSplineHeading(X, Math.toRadians(angle), drive.defaultVelConstraint, null);
                } else if (AccCon) {
                    lineToXSplineHeading = drive.actionBuilder(startingPose)
                            .lineToXSplineHeading(X, Math.toRadians(angle), null, drive.defaultAccelConstraint);
                } else {
                    lineToXSplineHeading = drive.actionBuilder(startingPose)
                            .lineToXSplineHeading(X, Math.toRadians(angle));
                }

                Actions.runBlocking(
                        new SequentialAction(
                                lineToXSplineHeading.build()));

                return(currentPose());
            }
            public static Pose2d lineToY(double Y, boolean VelCon, boolean AccCon, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder lineToY;
                if (VelCon && AccCon) {
                    lineToY = drive.actionBuilder(startingPose)
                            .lineToY(Y, drive.defaultVelConstraint, drive.defaultAccelConstraint);
                } else if (VelCon) {
                    lineToY = drive.actionBuilder(startingPose)
                            .lineToY(Y, drive.defaultVelConstraint, null);
                } else if (AccCon) {
                    lineToY = drive.actionBuilder(startingPose)
                            .lineToY(Y, null, drive.defaultAccelConstraint);
                } else {
                    lineToY = drive.actionBuilder(startingPose)
                            .lineToY(Y);
                }

                Actions.runBlocking(
                        new SequentialAction(
                                lineToY.build()));

                return(currentPose());
            }
            public static Pose2d lineToYConstantHeading(double Y, boolean VelCon, boolean AccCon, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder lineToYConstantHeading;
                if (VelCon && AccCon) {
                    lineToYConstantHeading = drive.actionBuilder(startingPose)
                            .lineToYConstantHeading(Y, drive.defaultVelConstraint, drive.defaultAccelConstraint);
                } else if (VelCon) {
                    lineToYConstantHeading = drive.actionBuilder(startingPose)
                            .lineToYConstantHeading(Y, drive.defaultVelConstraint, null);
                } else if (AccCon) {
                    lineToYConstantHeading = drive.actionBuilder(startingPose)
                            .lineToYConstantHeading(Y, null, drive.defaultAccelConstraint);
                } else {
                    lineToYConstantHeading = drive.actionBuilder(startingPose)
                            .lineToYConstantHeading(Y);
                }

                Actions.runBlocking(
                        new SequentialAction(
                                lineToYConstantHeading.build()));

                return(currentPose());
            }
            public static Pose2d lineToYLinearHeading(double Y, double angle, boolean VelCon, boolean AccCon, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder lineToYLinearHeading;
                if (VelCon && AccCon) {
                    lineToYLinearHeading = drive.actionBuilder(startingPose)
                            .lineToYLinearHeading(Y, Math.toRadians(angle), drive.defaultVelConstraint, drive.defaultAccelConstraint);
                } else if (VelCon) {
                    lineToYLinearHeading = drive.actionBuilder(startingPose)
                            .lineToYLinearHeading(Y, Math.toRadians(angle), drive.defaultVelConstraint, null);
                } else if (AccCon) {
                    lineToYLinearHeading = drive.actionBuilder(startingPose)
                            .lineToYLinearHeading(Y, Math.toRadians(angle), null, drive.defaultAccelConstraint);
                } else {
                    lineToYLinearHeading = drive.actionBuilder(startingPose)
                            .lineToYLinearHeading(Y, Math.toRadians(angle));
                }

                Actions.runBlocking(
                        new SequentialAction(
                                lineToYLinearHeading.build()));

                return(currentPose());
            }
            public static Pose2d lineToYSplineHeading(double Y, double angle, boolean VelCon, boolean AccCon, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder lineToYSplineHeading;
                if (VelCon && AccCon) {
                    lineToYSplineHeading = drive.actionBuilder(startingPose)
                            .lineToYSplineHeading(Y, Math.toRadians(angle), drive.defaultVelConstraint, drive.defaultAccelConstraint);
                } else if (VelCon) {
                    lineToYSplineHeading = drive.actionBuilder(startingPose)
                            .lineToYSplineHeading(Y, Math.toRadians(angle), drive.defaultVelConstraint, null);
                } else if (AccCon) {
                    lineToYSplineHeading = drive.actionBuilder(startingPose)
                            .lineToYSplineHeading(Y, Math.toRadians(angle), null, drive.defaultAccelConstraint);
                } else {
                    lineToYSplineHeading = drive.actionBuilder(startingPose)
                            .lineToYSplineHeading(Y, Math.toRadians(angle));
                }

                Actions.runBlocking(
                        new SequentialAction(
                                lineToYSplineHeading.build()));

                return(currentPose());
            }
        }

        public static Pose2d turnTo(double angle, Pose2d startingPose) {
            MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
            TrajectoryActionBuilder turnTo = drive.actionBuilder(startingPose)
                    .turnTo(angle);
            Actions.runBlocking(
                    new SequentialAction(
                            turnTo.build()));

            return(currentPose());

        }
        public static Pose2d turn(double angle, Pose2d startingPose) {
            MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
            TrajectoryActionBuilder turn = drive.actionBuilder(startingPose)
                    .turn(angle);
            Actions.runBlocking(
                    new SequentialAction(
                            turn.build()));

            return(currentPose());
        }
    }
    public static class utils {
        public utils(){
            super();
        }
        public static void ledManager(String type, int ledNumber){
            CRServo led = leds.get(ledNumber);
            if (led != null) {
                switch (type) {
                    case "Clear":
                        led.setPower(.5); //White

                        break;
                    case "Good":
                        led.setPower(0); //Green

                        break;
                    case "Warn":
                        led.setPower(-.25); //Yellow

                        break;
                    case "Alert":
                        led.setPower(-.35); //Orange

                        break;
                    case "Error":
                        led.setPower(-0.44); //red

                        break;
                    case "Null":
                        led.setPower(-.6); //Blank

                        break;
                    case "Match Alert":
                        led.setPower(0); //Purple

                        break;
                    case "Blue":
                        led.setPower(0.216); //Blue

                        break;
                    case "Purple":
                        led.setPower(0.415); //Purple

                        break;
                    case "Pink":
                        led.setPower(0.275); //Pink

                        break;
                    default:
                        if(debug.debugTelemetry) {
                            telemetry.addData("Led Manager Error", "Wrong or Invalid Input");
                        }
                        break;
                }
            }
        }
    }
}