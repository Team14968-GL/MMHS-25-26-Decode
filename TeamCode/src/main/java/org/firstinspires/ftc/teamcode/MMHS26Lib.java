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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class MMHS26Lib {
    //Hardware variables
    private static DcMotor leftBack;
    private static DcMotor rightBack;
    private static DcMotor leftFront;
    private static DcMotor rightFront;
    private static GoBildaPinpointDriver pinpoint;
    private static CRServo LED1;
    private static ArrayList<CRServo> leds;
    private static Limelight3A limelight;
    private static DcMotor intakeMotor;
    private static Servo goofyAhhhhFrontDoor;
    private static DcMotor leftLauncher;
    private static DcMotor rightLauncher;
    private static DcMotor lift;
    private static CRServo launchLiftRight;
    private static CRServo launchLiftLeft;
    private static Servo backDoor;
    private static Servo scoop;
    private static Servo turnTableServo;
    private static TouchSensor TopBump;
    private static TouchSensor BottomBump;
    private static TouchSensor intakeBump1;
    private static TouchSensor intakeBump2;

    //Constants
    private static final double ticPerIn = 254.7;

    //Internal variables
    private static HardwareMap hwMap;

    public MMHS26Lib(HardwareMap hardwareMap) {
        //Drive Definitions
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        //Intake Definitions
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        goofyAhhhhFrontDoor = hardwareMap.get(Servo.class, "goofyAhhhhFrontDoor");
        intakeBump1 = hardwareMap.get(TouchSensor.class, "intakeBump1");
        intakeBump2 = hardwareMap.get(TouchSensor.class, "intakeBump2");
        //Launcher Definitions
        leftLauncher = hardwareMap.get(DcMotor.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotor.class, "rightLauncher");
        launchLiftRight = hardwareMap.get(CRServo.class, "launchLiftRight");
        launchLiftLeft = hardwareMap.get(CRServo.class, "launchLiftLeft");
        TopBump = hardwareMap.get(TouchSensor.class, "TopBump");
        BottomBump = hardwareMap.get(TouchSensor.class, "BottomBump");
        backDoor = hardwareMap.get(Servo.class, "backDoor");
        scoop = hardwareMap.get(Servo.class, "scoop");
        //Lift/Skis Definition
        lift = hardwareMap.get(DcMotor.class, "lift");
        //Turntable Definition
        turnTableServo = hardwareMap.get(Servo.class, "turnTableServo");
        //Drive Config
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //Intake Config
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        //Launcher Config
        leftLauncher.setDirection(DcMotor.Direction.REVERSE);
        rightLauncher.setDirection(DcMotor.Direction.FORWARD);
        leftLauncher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchLiftRight.setDirection(CRServo.Direction.REVERSE);
        launchLiftLeft.setDirection(CRServo.Direction.FORWARD);
        //Lift/Skis Config
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Odometry Config
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.initialize(); //Initializes odometry for use in code
        //LED Config
        LED1 = hardwareMap.get(CRServo.class, "Led1");
        leds = new ArrayList<>(Arrays.asList(null, LED1)); //creates a list of LEDs for ledManager to use
        //Limelight Config/Setup
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); //Sets the config the limelight should use
        limelight.setPollRateHz(100); //Limelight data polling rate
        limelight.start(); //Initializes limelight for use in code

        //Internal Hardware Map (DO NOT TOUCH)
        hwMap = hardwareMap;
    }

    //Optional flag(s) to enable internal debugging tools
    @Config
    public static class debug {
        public static boolean debugTelemetry = false;
    }

    //Sleep function taken from LinearOpMode (importing this from LinearOpMode doesn't work)
    private static void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    //Gets the robots current position on the field
    public static Pose2d currentPose() {
        return (new Pose2d(new Vector2d(pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH)), pinpoint.getHeading(AngleUnit.RADIANS)));
    }

    //Class for managing basic functions relating to movement
    public static class motion {
        public motion() {super();}
        //DO NOT USE STRAFE UNLESS NEEDED
        public static void strafeLeft(double Speed, long time) {
            leftBack.setPower(Speed);
            leftFront.setPower(-Speed);
            rightBack.setPower(Speed);
            rightFront.setPower(-Speed);
            sleep(time);
            leftBack.setPower(0);
            leftFront.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);
        }
        //DO NOT USE STRAFE UNLESS NEEDED
        public static void strafeRight(double Speed, long time) {
            leftBack.setPower(-Speed);
            leftFront.setPower(Speed);
            rightBack.setPower(-Speed);
            rightFront.setPower(Speed);
            sleep(time);
            leftBack.setPower(0);
            leftFront.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);
        }

        public static void turnRight(double Speed, long time) {
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

        public static void turnLeft(double Speed, long time) {
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

        public static void moveBackward(double Speed, long time) {
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

        public static void moveForward(double Speed, long time) {
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

        public static void halt() {
            leftBack.setPower(0);
            leftFront.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);
        }
    }

    //Class for limelight functions
    public static class Limelight {
        public Limelight() {super();}

        //moves to a predetermined point on the field
        public static void localizer(double localPower, int sleepTime) {
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
                    if (debug.debugTelemetry) {
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
                        //move forward
                        motion.moveForward(localPower, sleepTime);
                    } else {
                        break;
                    }
                } else {
                    if (debug.debugTelemetry) {
                        telemetry.addData("Limelight", "No Targets");
                        telemetry.update();
                    }
                    //stop movement
                    motion.halt();
                }
            }
        }

        //Outputs a list of AprilTag IDs that the limelight can see
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

                if (debug.debugTelemetry) {
                    telemetry.addData("Target X", tx);
                    telemetry.addData("Target Y", ty);
                    telemetry.addData("Target Area", ta);
                    telemetry.update();
                }

                if (!fiducialList.isEmpty()) {
                    if (debug.debugTelemetry) {
                        telemetry.addData("Detections Found", fiducialList.size());
                        telemetry.update();
                    }
                    // Iterate through each detected tag
                    for (LLResultTypes.FiducialResult fiducial : fiducialList) {
                        id = fiducial.getFiducialId();
                        IDs.add(id);
                        if (debug.debugTelemetry) {
                            telemetry.addData("Tag ID", id);
                            telemetry.update();
                        }
                    }
                } else {
                    if (debug.debugTelemetry) {
                        telemetry.addData("Detections Found", "None");
                        telemetry.update();
                    }
                }
            } else {
                if (debug.debugTelemetry) {
                    telemetry.addData("Limelight Data", "Invalid or Stale");
                    assert result != null;
                    telemetry.addData("Staleness", result.getStaleness());
                    telemetry.update();
                }
            }
            if (debug.debugTelemetry) {
                telemetry.update();
            }
            return id;
        }
        public static Pose2d poseLimelight() {

            LLResult result = limelight.getLatestResult();
            Pose3D pose;
            double x = 0;
            double y = 0;
            double yaw = 0;

            if (result != null && result.isValid()) {
                pose = result.getBotpose();

                telemetry.addData("X", (pose.getPosition().x * 39.37));
                telemetry.addData("Y",  (pose.getPosition().y * 39.37));
                telemetry.addData("Rotation",  pose.getOrientation().getYaw());
                telemetry.update();

                return new Pose2d((pose.getPosition().x * 39.37), (pose.getPosition().y * 39.37), (pose.getOrientation().getYaw() - 180));
            } else {
                telemetry.addData("Limelight", "Failed to localize, defaulting to X:0 Y:0 θ:0");
                return new Pose2d(0, 0, 0);
            }
        }
    }

    //Class for functions relating to the autonomous pathing tool RoadRunner
    public static class roadRunner {
        public roadRunner() {super();}

        //Creates a curved path for the robot to automatically follow
        public static class spline {
            public spline() {super();}

            public static Pose2d splineTo(double x, double y, double tangent, Pose2d startingPose) {

                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder splineTo = drive.actionBuilder(startingPose)
                        .splineTo(new Vector2d(x, y), Math.toRadians(tangent));
                Actions.runBlocking(
                        new SequentialAction(
                                splineTo.build()));

                return (currentPose());
            }

            public static Pose2d splineToConstantHeading(double x, double y, double tangent, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder splineToConstantHeading = drive.actionBuilder(startingPose)
                        .splineToConstantHeading(new Vector2d(x, y), Math.toRadians(tangent));
                Actions.runBlocking(
                        new SequentialAction(
                                splineToConstantHeading.build()));


                return (currentPose());
            }

            public static Pose2d splineToLinearHeading(double x, double y, double angle, double tangent, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder splineToLinearHeading = drive.actionBuilder(startingPose)
                        .splineToLinearHeading(new Pose2d(new Vector2d(x, y), angle), tangent);
                Actions.runBlocking(
                        new SequentialAction(
                                splineToLinearHeading.build()));


                return (currentPose());
            }

            public static Pose2d splineToSplineHeading(double x, double y, double angle, double tangent, Pose2d startingPose) {
                MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
                TrajectoryActionBuilder splineToSplineHeading = drive.actionBuilder(startingPose)
                        .splineToSplineHeading(new Pose2d(new Vector2d(x, y), angle), tangent);
                Actions.runBlocking(
                        new SequentialAction(
                                splineToSplineHeading.build()));

                return (currentPose());
            }
        }

        //Creates a horizontal path for the robot to move along
        public static class strafe {
            public strafe() {super();}

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

                return (currentPose());
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

                return (currentPose());
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

                return (currentPose());
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

                return (currentPose());
            }
        }

        //takes a line to a point on a specified axis (x or y)
        public static class lineTo {
            public lineTo() {super();}

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

                return (currentPose());
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

                return (currentPose());
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

                return (currentPose());
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

                return (currentPose());
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

                return (currentPose());
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

                return (currentPose());
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

                return (currentPose());
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

                return (currentPose());
            }
        }

        //turns to a specified angle
        public static Pose2d turnTo(double angle, Pose2d startingPose) {
            MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
            TrajectoryActionBuilder turnTo = drive.actionBuilder(startingPose)
                    .turnTo(angle);
            Actions.runBlocking(
                    new SequentialAction(
                            turnTo.build()));

            return (currentPose());

        }

        //turns to a specified angle
        public static Pose2d turn(double angle, Pose2d startingPose) {
            MecanumDrive drive = new MecanumDrive(hwMap, startingPose);
            TrajectoryActionBuilder turn = drive.actionBuilder(startingPose)
                    .turn(angle);
            Actions.runBlocking(
                    new SequentialAction(
                            turn.build()));

            return (currentPose());
        }
    }

    //Class for non-critical utility systems
    public static class utils {
        public utils() {super();}
        //Class for managing telemetry systems
        public static class telemetrySys {
            telemetrySys() {
                super();
            }

            public static class motor {
                motor() {super();}
                public static void leftBackMotor() {
                    telemetry.addData("Left Back", leftBack);
                    telemetry.addData("Left Back", leftBack.getConnectionInfo());
                    telemetry.addData("Left Back", leftBack.getDeviceName());
                    telemetry.addData("Left Back", leftBack.getDirection());
                    telemetry.addData("Left Back", leftBack.getManufacturer());
                    telemetry.addData("Left Back", leftBack.getVersion());
                    telemetry.addData("Left Back", leftBack.getClass());
                    telemetry.addData("Left Back", leftBack.isBusy());
                    telemetry.addData("Left Back", leftBack.getController());
                    telemetry.addData("Left Back", leftBack.getMotorType());
                    telemetry.addData("Left Back", leftBack.getCurrentPosition());
                    telemetry.addData("Left Back", leftBack.getMode());
                    telemetry.addData("Left Back", leftBack.getPortNumber());
                    telemetry.addData("Left Back", leftBack.getPowerFloat());
                    telemetry.addData("Left Back", leftBack.getTargetPosition());
                    telemetry.addData("Left Back", leftBack.getZeroPowerBehavior());
                }
                public static void rightBackMotor() {
                    telemetry.addData("Right Back", rightBack);
                    telemetry.addData("Right Back", rightBack.getConnectionInfo());
                    telemetry.addData("Right Back", rightBack.getDeviceName());
                    telemetry.addData("Right Back", rightBack.getDirection());
                    telemetry.addData("Right Back", rightBack.getManufacturer());
                    telemetry.addData("Right Back", rightBack.getVersion());
                    telemetry.addData("Right Back", rightBack.getClass());
                    telemetry.addData("Right Back", rightBack.isBusy());
                    telemetry.addData("Right Back", rightBack.getController());
                    telemetry.addData("Right Back", rightBack.getMotorType());
                    telemetry.addData("Right Back", rightBack.getCurrentPosition());
                    telemetry.addData("Right Back", rightBack.getMode());
                    telemetry.addData("Right Back", rightBack.getPortNumber());
                    telemetry.addData("Right Back", rightBack.getPowerFloat());
                    telemetry.addData("Right Back", rightBack.getTargetPosition());
                    telemetry.addData("Right Back", rightBack.getZeroPowerBehavior());
                }
                public static void rightFrontMotor() {
                    telemetry.addData("Right Front", rightFront);
                    telemetry.addData("Right Front", rightFront.getConnectionInfo());
                    telemetry.addData("Right Front", rightFront.getDeviceName());
                    telemetry.addData("Right Front", rightFront.getDirection());
                    telemetry.addData("Right Front", rightFront.getManufacturer());
                    telemetry.addData("Right Front", rightFront.getVersion());
                    telemetry.addData("Right Front", rightFront.getClass());
                    telemetry.addData("Right Front", rightFront.isBusy());
                    telemetry.addData("Right Front", rightFront.getController());
                    telemetry.addData("Right Front", rightFront.getMotorType());
                    telemetry.addData("Right Front", rightFront.getCurrentPosition());
                    telemetry.addData("Right Front", rightFront.getMode());
                    telemetry.addData("Right Front", rightFront.getPortNumber());
                    telemetry.addData("Right Front", rightFront.getPowerFloat());
                    telemetry.addData("Right Front", rightFront.getTargetPosition());
                    telemetry.addData("Right Front", rightFront.getZeroPowerBehavior());
                }
                public static void leftFrontMotor() {
                    telemetry.addData("Left Front", leftFront);
                    telemetry.addData("Left Front", leftFront.getConnectionInfo());
                    telemetry.addData("Left Front", leftFront.getDeviceName());
                    telemetry.addData("Left Front", leftFront.getDirection());
                    telemetry.addData("Left Front", leftFront.getManufacturer());
                    telemetry.addData("Left Front", leftFront.getVersion());
                    telemetry.addData("Left Front", leftFront.getClass());
                    telemetry.addData("Left Front", leftFront.isBusy());
                    telemetry.addData("Left Front", leftFront.getController());
                    telemetry.addData("Left Front", leftFront.getMotorType());
                    telemetry.addData("Left Front", leftFront.getCurrentPosition());
                    telemetry.addData("Left Front", leftFront.getMode());
                    telemetry.addData("Left Front", leftFront.getPortNumber());
                    telemetry.addData("Left Front", leftFront.getPowerFloat());
                    telemetry.addData("Left Front", leftFront.getTargetPosition());
                    telemetry.addData("Left Front", leftFront.getZeroPowerBehavior());
                }
                public static void leftLauncherMotor() {
                    telemetry.addData("Left Launcher", leftLauncher);
                    telemetry.addData("Left Launcher", leftLauncher.getConnectionInfo());
                    telemetry.addData("Left Launcher", leftLauncher.getDeviceName());
                    telemetry.addData("Left Launcher", leftLauncher.getDirection());
                    telemetry.addData("Left Launcher", leftLauncher.getManufacturer());
                    telemetry.addData("Left Launcher", leftLauncher.getVersion());
                    telemetry.addData("Left Launcher", leftLauncher.getClass());
                    telemetry.addData("Left Launcher", leftLauncher.isBusy());
                    telemetry.addData("Left Launcher", leftLauncher.getController());
                    telemetry.addData("Left Launcher", leftLauncher.getMotorType());
                    telemetry.addData("Left Launcher", leftLauncher.getCurrentPosition());
                    telemetry.addData("Left Launcher", leftLauncher.getMode());
                    telemetry.addData("Left Launcher", leftLauncher.getPortNumber());
                    telemetry.addData("Left Launcher", leftLauncher.getPowerFloat());
                    telemetry.addData("Left Launcher", leftLauncher.getTargetPosition());
                    telemetry.addData("Left Launcher", leftLauncher.getZeroPowerBehavior());
                }
                public static void rightLauncherMotor() {
                    telemetry.addData("Right Launcher", rightLauncher);
                    telemetry.addData("Right Launcher", rightLauncher.getConnectionInfo());
                    telemetry.addData("Right Launcher", rightLauncher.getDeviceName());
                    telemetry.addData("Right Launcher", rightLauncher.getDirection());
                    telemetry.addData("Right Launcher", rightLauncher.getManufacturer());
                    telemetry.addData("Right Launcher", rightLauncher.getVersion());
                    telemetry.addData("Right Launcher", rightLauncher.getClass());
                    telemetry.addData("Right Launcher", rightLauncher.isBusy());
                    telemetry.addData("Right Launcher", rightLauncher.getController());
                    telemetry.addData("Right Launcher", rightLauncher.getMotorType());
                    telemetry.addData("Right Launcher", rightLauncher.getCurrentPosition());
                    telemetry.addData("Right Launcher", rightLauncher.getMode());
                    telemetry.addData("Right Launcher", rightLauncher.getPortNumber());
                    telemetry.addData("Right Launcher", rightLauncher.getPowerFloat());
                    telemetry.addData("Right Launcher", rightLauncher.getTargetPosition());
                    telemetry.addData("Right Launcher", rightLauncher.getZeroPowerBehavior());
                }
                public static void liftMotor() {
                    telemetry.addData("Lift", lift);
                    telemetry.addData("Lift", lift.getConnectionInfo());
                    telemetry.addData("Lift", lift.getDeviceName());
                    telemetry.addData("Lift", lift.getDirection());
                    telemetry.addData("Lift", lift.getManufacturer());
                    telemetry.addData("Lift", lift.getVersion());
                    telemetry.addData("Lift", lift.getClass());
                    telemetry.addData("Lift", lift.isBusy());
                    telemetry.addData("Lift", lift.getController());
                    telemetry.addData("Lift", lift.getMotorType());
                    telemetry.addData("Lift", lift.getCurrentPosition());
                    telemetry.addData("Lift", lift.getMode());
                    telemetry.addData("Lift", lift.getPortNumber());
                    telemetry.addData("Lift", lift.getPowerFloat());
                    telemetry.addData("Lift", lift.getTargetPosition());
                    telemetry.addData("Lift", lift.getZeroPowerBehavior());
                }
                public static void intakeMotor() {
                    telemetry.addData("Intake", intakeMotor);
                    telemetry.addData("Intake", intakeMotor.getConnectionInfo());
                    telemetry.addData("Intake", intakeMotor.getDeviceName());
                    telemetry.addData("Intake", intakeMotor.getDirection());
                    telemetry.addData("Intake", intakeMotor.getManufacturer());
                    telemetry.addData("Intake", intakeMotor.getVersion());
                    telemetry.addData("Intake", intakeMotor.getClass());
                    telemetry.addData("Intake", intakeMotor.isBusy());
                    telemetry.addData("Intake", intakeMotor.getController());
                    telemetry.addData("Intake", intakeMotor.getMotorType());
                    telemetry.addData("Intake", intakeMotor.getCurrentPosition());
                    telemetry.addData("Intake", intakeMotor.getMode());
                    telemetry.addData("Intake", intakeMotor.getPortNumber());
                    telemetry.addData("Intake", intakeMotor.getPowerFloat());
                    telemetry.addData("Intake", intakeMotor.getTargetPosition());
                    telemetry.addData("Intake", intakeMotor.getZeroPowerBehavior());
                }
            }
            public static class servos {
                servos() {super();}
                public static void turnTable() {
                    telemetry.addData("Turn Table", turnTableServo);
                    telemetry.addData("Turn Table", turnTableServo.getController());
                    telemetry.addData("Turn Table", turnTableServo.getDirection());
                    telemetry.addData("Turn Table", turnTableServo.getPortNumber());
                    telemetry.addData("Turn Table", turnTableServo.getPosition());
                    telemetry.addData("Turn Table", turnTableServo.getConnectionInfo());
                    telemetry.addData("Turn Table", turnTableServo.getDeviceName());
                    telemetry.addData("Turn Table", turnTableServo.getManufacturer());
                    telemetry.addData("Turn Table", turnTableServo.getVersion());
                    telemetry.addData("Turn Table", turnTableServo.getClass());
                }

                public static void frontDoor() {
                    telemetry.addData("Front Door", goofyAhhhhFrontDoor);
                    telemetry.addData("Front Door", goofyAhhhhFrontDoor.getController());
                    telemetry.addData("Front Door", goofyAhhhhFrontDoor.getDirection());
                    telemetry.addData("Front Door", goofyAhhhhFrontDoor.getPortNumber());
                    telemetry.addData("Front Door", goofyAhhhhFrontDoor.getPosition());
                    telemetry.addData("Front Door", goofyAhhhhFrontDoor.getConnectionInfo());
                    telemetry.addData("Front Door", goofyAhhhhFrontDoor.getDeviceName());
                    telemetry.addData("Front Door", goofyAhhhhFrontDoor.getManufacturer());
                    telemetry.addData("Front Door", goofyAhhhhFrontDoor.getVersion());
                    telemetry.addData("Front Door", goofyAhhhhFrontDoor.getClass());
                }
                public static void launchDoor() {
                    telemetry.addData("Back Door", backDoor);
                    telemetry.addData("Back Door", backDoor.getController());
                    telemetry.addData("Back Door", backDoor.getDirection());
                    telemetry.addData("Back Door", backDoor.getPortNumber());
                    telemetry.addData("Back Door", backDoor.getPosition());
                    telemetry.addData("Back Door", backDoor.getConnectionInfo());
                    telemetry.addData("Back Door", backDoor.getDeviceName());
                    telemetry.addData("Back Door", backDoor.getManufacturer());
                    telemetry.addData("Back Door", backDoor.getVersion());
                    telemetry.addData("Back Door", backDoor.getClass());
                }
                public static void launchScoop() {
                    telemetry.addData("Scoop", scoop);
                    telemetry.addData("Scoop", scoop.getController());
                    telemetry.addData("Scoop", scoop.getDirection());
                    telemetry.addData("Scoop", scoop.getPortNumber());
                    telemetry.addData("Scoop", scoop.getPosition());
                    telemetry.addData("Scoop", scoop.getConnectionInfo());
                    telemetry.addData("Scoop", scoop.getDeviceName());
                    telemetry.addData("Scoop", scoop.getManufacturer());
                    telemetry.addData("Scoop", scoop.getVersion());
                    telemetry.addData("Scoop", scoop.getClass());
                }
                public static void leftLaunchLift() {
                    telemetry.addData("Left Launch Lift", launchLiftLeft);
                    telemetry.addData("Left Launch Lift", launchLiftLeft.getController());
                    telemetry.addData("Left Launch Lift", launchLiftLeft.getDirection());
                    telemetry.addData("Left Launch Lift", launchLiftLeft.getPortNumber());
                    telemetry.addData("Left Launch Lift", launchLiftLeft.getPower());
                    telemetry.addData("Left Launch Lift", launchLiftLeft.getConnectionInfo());
                    telemetry.addData("Left Launch Lift", launchLiftLeft.getDeviceName());
                    telemetry.addData("Left Launch Lift", launchLiftLeft.getManufacturer());
                    telemetry.addData("Left Launch Lift", launchLiftLeft.getVersion());
                    telemetry.addData("Left Launch Lift", launchLiftLeft.getClass());
                }
                public static void rightLaunchLift() {
                    telemetry.addData("Right Launch Lift", launchLiftRight);
                    telemetry.addData("Right Launch Lift", launchLiftRight.getController());
                    telemetry.addData("Right Launch Lift", launchLiftRight.getDirection());
                    telemetry.addData("Right Launch Lift", launchLiftRight.getPortNumber());
                    telemetry.addData("Right Launch Lift", launchLiftRight.getPower());
                    telemetry.addData("Right Launch Lift", launchLiftRight.getConnectionInfo());
                    telemetry.addData("Right Launch Lift", launchLiftRight.getDeviceName());
                    telemetry.addData("Right Launch Lift", launchLiftRight.getManufacturer());
                    telemetry.addData("Right Launch Lift", launchLiftRight.getVersion());
                    telemetry.addData("Right Launch Lift", launchLiftRight.getClass());
                }
            }
        }
        //Function to manage the color of LED(s) on the robot
        public static void ledManager(String type, int ledNumber) {
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
                        if (debug.debugTelemetry) {
                            telemetry.addData("Led Manager Error", "Wrong or Invalid Input");
                            RobotLog.ii("Led Manager Error", "Wrong or Invalid Input");
                        }
                        break;
                }
            }
        }

        public static class auto {
            public auto() {
                super();
            }

            public void intake3Balls(double searchSpeed, double returnSpeed, double returnDistance, int kickTime) {
                int safeTrig;
                turnTableServo.setPosition(0);
                goofyAhhhhFrontDoor.setPosition(1);
                intakeMotor.setPower(0.8);
                safeTrig = BackwardsTillBump(searchSpeed, 0);
                if (safeTrig == 1) {
                    moveForwardTics(returnSpeed, returnDistance * ticPerIn);
                    halfKick(kickTime);
                    sleep(250);
                    turnTableServo.setPosition(0.5);
                    goofyAhhhhFrontDoor.setPosition(1);

                    safeTrig = BackwardsTillBump(searchSpeed, 0);
                    if (safeTrig == 1) {
                        moveForwardTics(returnSpeed, returnDistance * ticPerIn);
                        halfKick(kickTime);
                        sleep(250);
                        turnTableServo.setPosition(1);
                        goofyAhhhhFrontDoor.setPosition(1);
                        safeTrig = BackwardsTillBump(searchSpeed, 0);
                        if (safeTrig == 1) {
                            halfKick(kickTime);
                            intakeMotor.setPower(0);

                        } else {

                            goofyAhhhhFrontDoor.setPosition(.5);
                            intakeMotor.setPower(0);
                        }

                    } else {
                        goofyAhhhhFrontDoor.setPosition(.5);
                        intakeMotor.setPower(0);
                    }
                } else {
                    goofyAhhhhFrontDoor.setPosition(.5);
                    intakeMotor.setPower(0);
                }
            }

            private void moveForwardTics(double Speed, double tic) {
                pinpoint.update();
                double xvalue = pinpoint.getEncoderX();
                while (xvalue - pinpoint.getEncoderX() <= tic) {
                    pinpoint.update();
                    leftBack.setPower(-Speed);
                    leftFront.setPower(-Speed);
                    rightBack.setPower(-Speed);
                    rightFront.setPower(-Speed);
                }
                leftBack.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
            }

            public int BackwardsTillBump(double Speed, int delay) {
                int count = 0;
                int returnSave = 2;

                ElapsedTime BackwardsTillBumpClock = new ElapsedTime();

                BackwardsTillBumpClock.reset();
                while (BackwardsTillBumpClock.seconds() <= 2 && !(!intakeBump1.isPressed() || intakeBump2.isPressed())) {
                    leftBack.setPower(Speed);
                    leftFront.setPower(Speed);
                    rightBack.setPower(Speed);
                    rightFront.setPower(Speed);
                    sleep(1);
                    count++;
                }
                if (!intakeBump1.isPressed() || intakeBump2.isPressed()) {
                    returnSave = 1;
                }

                if (BackwardsTillBumpClock.seconds() >= 2) {
                    returnSave = 0;
                } else {
                    returnSave = 1;
                }


                leftBack.setPower(0);
                leftFront.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);
                return returnSave;
            }
            public void halfKick(int time) {
                goofyAhhhhFrontDoor.setPosition(0);
                sleep(time);
                goofyAhhhhFrontDoor.setPosition(.5);
            }
        }
    }
}