package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "Test XY Limelight")
public class TestXYLimelight extends LinearOpMode {

    private Limelight3A limelight;
    private test_robot tr;

    private static final int TARGET_TAG_ID = 30;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );
        tr=new test_robot(hardwareMap);

        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result == null || !result.isValid()) {
                telemetry.addLine("No valid Limelight result");
                telemetry.update();
                continue;
            }

            List<LLResultTypes.FiducialResult> tags =
                    result.getFiducialResults();

            LLResultTypes.FiducialResult targetTag = null;

            // Find AprilTag ID 30
            for (LLResultTypes.FiducialResult tag : tags) {

                if (tag.getFiducialId() == TARGET_TAG_ID) {
                    targetTag = tag;
                    break;
                }
            }

            if (targetTag == null) {

                telemetry.addLine("AprilTag 30 not detected");
                telemetry.update();
                continue;
            }

            // Get target position
            Pose3D targetPose =
                    targetTag.getTargetPoseCameraSpace();

            if (targetPose == null) {
                telemetry.addLine("Target pose unavailable");
                telemetry.update();
                continue;
            }

            double x = targetPose.getPosition().x;
            double y = targetPose.getPosition().y;
            double z = targetPose.getPosition().z;

            // Convert X/Z position into horizontal angle
            double horizontalAngle =
                    Math.toDegrees(Math.atan2(x, z));

            // Convert Y/Z position into vertical angle
            double verticalAngle =
                    Math.toDegrees(Math.atan2(y, z));

            telemetry.addData(
                    "Horizontal Angle",
                    "%.2f°",
                    horizontalAngle
            );

            telemetry.addData(
                    "Vertical Angle",
                    "%.2f°",
                    verticalAngle
            );

            telemetry.addData(
                    "X",
                    "%.3f",
                    x
            );

            telemetry.addData(
                    "Y",
                    "%.3f",
                    y
            );

            telemetry.addData(
                    "Z",
                    "%.3f",
                    z
            );

            // ==========================================
            // STRAFE LOGIC
            // ==========================================

            if (horizontalAngle > 2.0) {
                tr.driveRight(0.2);
                telemetry.addLine("→ STRAFE RIGHT");

            } else if (horizontalAngle < -2.0) {
                tr.driveLeft(0.2);
                telemetry.addLine("← STRAFE LEFT");

            } else {
                tr.stopDrive();
                telemetry.addLine("✓ CENTERED - STOP");

            }

            telemetry.update();
        }
    }
}