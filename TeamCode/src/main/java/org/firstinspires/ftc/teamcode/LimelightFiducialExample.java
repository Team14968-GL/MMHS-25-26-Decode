package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;
@Disabled
@TeleOp(name = "LimelightFiducialExample", group = "Robot")
public class LimelightFiducialExample extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        // Initialize the Limelight object from the hardware map
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.addData("Status", "Limelight Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ... process results in the loop
            processFiducialResults();
            sleep(20); // Optional sleep to reduce CPU load
        }
    }
    private void processFiducialResults() {
        // Get the single latest result object from the Limelight
        LLResult result = limelight.getLatestResult();

        // Check if the result is valid and contains data
        if (result != null && result.isValid()) {
            // Get the list of ALL detected fiducials (AprilTags)
            List<LLResultTypes.FiducialResult> fiducialList = result.getFiducialResults();

            if (!fiducialList.isEmpty()) {
                telemetry.addData("Detections Found", fiducialList.size());

                // Iterate through each detected tag
                for (LLResultTypes.FiducialResult fiducial : fiducialList) {
                    int id = fiducial.getFiducialId();


                    // You can also get pose data (X, Y, Z, Pitch, Yaw, Roll)


                    telemetry.addData("Tag ID", id);

                }
            } else {
                telemetry.addData("Detections Found", "None");
            }
        } else {
            telemetry.addData("Limelight Data", "Invalid or Stale");
            telemetry.addData("Staleness",result.getStaleness());
        }
        telemetry.update();
    }


    // ... function to process results

}
