package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "BIOBUZZ - Limelight Tag Location", group = "Test")
public class LimelightTagLocationTest extends LinearOpMode {

    // Limelight hardware
    private Limelight3A limelight;

    @Override
    public void runOpMode() {

        // --------------------------------------------------
        // HARDWARE INITIALIZATION
        // --------------------------------------------------

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Select Limelight pipeline 0
        limelight.pipelineSwitch(0);

        // Start Limelight
        limelight.start();

        telemetry.addLine("BIOBUZZ LIMELIGHT TEST");
        telemetry.addLine("----------------------");
        telemetry.addLine("Limelight Initialized");
        telemetry.update();

        waitForStart();

        // --------------------------------------------------
        // MAIN LOOP
        // --------------------------------------------------

        while (opModeIsActive()) {

            // Get latest result from Limelight
            LLResult result = limelight.getLatestResult();

            telemetry.clearAll();

            telemetry.addLine("================================");
            telemetry.addLine("      BIOBUZZ LIMELIGHT");
            telemetry.addLine("================================");

            // Check if Limelight has a valid result
            if (result != null && result.isValid()) {

                // Get all detected AprilTags
                List<LLResultTypes.FiducialResult> fiducials =
                        result.getFiducialResults();

                // Check if any AprilTags are detected
                if (fiducials != null && !fiducials.isEmpty()) {

                    telemetry.addLine("AprilTag Detected: YES");
                    telemetry.addData("Tags Detected", fiducials.size());

                    telemetry.addLine("--------------------------------");

                    // Process every detected tag
                    for (LLResultTypes.FiducialResult tag : fiducials) {

                        // Get AprilTag ID
                        int tagId = tag.getFiducialId();

                        // Determine tag location
                        String location = getTagLocation(tagId);

                        telemetry.addData(
                                "Tag ID",
                                tagId
                        );

                        telemetry.addData(
                                "Location",
                                location
                        );

                        telemetry.addLine("--------------------------------");
                    }

                } else {

                    // No AprilTag detected
                    telemetry.addLine("AprilTag Detected: NO");
                    telemetry.addData("Tag ID", "NONE");
                    telemetry.addData("Location", "UNKNOWN");
                }

            } else {

                // Limelight does not have a valid target/result
                telemetry.addLine("AprilTag Detected: NO");
                telemetry.addData("Tag ID", "NONE");
                telemetry.addData("Location", "UNKNOWN");
            }

            telemetry.update();

            // Small delay to prevent unnecessary loop speed
            sleep(20);
        }

        // Stop Limelight when OpMode ends
        limelight.stop();
    }

    /**
     * Determines the BIOBUZZ location based on AprilTag ID.
     *
     * Tags 30-33:
     *      Opposite of Audience
     *
     * Tags 34-37:
     *      Audience Side
     *
     * Any other tag:
     *      Unknown
     */
    private String getTagLocation(int tagId) {

        // Tags 30, 31, 32, 33
        if (tagId >= 30 && tagId <= 33) {

            return "OPPOSITE OF AUDIENCE";

        }

        // Tags 34, 35, 36, 37
        else if (tagId >= 34 && tagId <= 37) {

            return "AUDIENCE SIDE";

        }

        // Any tag outside BIOBUZZ range
        else {

            return "UNKNOWN";
        }
    }
}