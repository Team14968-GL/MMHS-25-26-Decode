package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;

@TeleOp(name = "Limelight AprilTag ID Test", group = "Test")
public class LimelightAprilTagTest extends LinearOpMode {

    // CHANGE THIS: Limelight's IP address
    private final String LIMELIGHT_IP = "http://172.29.0.30:5800";

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Ready to read AprilTags from Limelight...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            try {
                // --- Read LL3 JSON pipeline data ---
                URL url = new URL(LIMELIGHT_IP + "/json");
                HttpURLConnection conn = (HttpURLConnection) url.openConnection();
                conn.setRequestMethod("GET");

                BufferedReader in = new BufferedReader(
                        new InputStreamReader(conn.getInputStream())
                );

                StringBuilder response = new StringBuilder();
                String line;

                while ((line = in.readLine()) != null) {
                    response.append(line);
                }
                in.close();

                // --- Parse JSON ---
                JSONObject obj = new JSONObject(response.toString());

                JSONArray results = obj.getJSONArray("Results");

                if (results.length() > 0) {
                    JSONObject tag = results.getJSONObject(0);

                    int id = tag.getInt("tid"); // APRILTAG ID

                    //telemetry.addData("AprilTag ID", id);
                    telemetry.addData("tag",tag);
                } else {
                    telemetry.addLine("No AprilTag Detected");
                }

            } catch (Exception e) {
                telemetry.addData("Error", e.getMessage());
            }

            telemetry.update();
        }
    }
}
