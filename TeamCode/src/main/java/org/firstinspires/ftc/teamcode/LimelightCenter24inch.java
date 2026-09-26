package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "AprilTag Center + 24 Inch")
public class LimelightCenter24inch extends LinearOpMode {

    private Limelight3A limelight;
    private test_robot tr;

    // ==========================================
    // APRILTAG
    // ==========================================

    private static final int TARGET_TAG_ID = 30;

    // ==========================================
    // TARGET DISTANCE
    // ==========================================

    private static final double TARGET_DISTANCE_INCH = 30.0;
    //0 center
    //1 forward
    private static int STATE_MANAGE=0;

    // Allowed distance error
    private static final double DISTANCE_TOLERANCE = 0.5;

    // Allowed horizontal angle error
    private static final double ANGLE_TOLERANCE = 2.0;

    // ==========================================
    // DISTANCE CALIBRATION
    // ==========================================

    // Your previous calibration
    private static final double DISTANCE_OFFSET_INCH = 5.0;

    @Override
    public void runOpMode() throws InterruptedException {
        STATE_MANAGE = 0;

        // ==========================================
        // INITIALIZE LIMELIGHT
        // ==========================================

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        // ==========================================
        // INITIALIZE ROBOT
        // ==========================================

        tr = new test_robot(hardwareMap);

        // ==========================================
        // APRILTAG PIPELINE
        // ==========================================

        limelight.pipelineSwitch(0);

        limelight.start();

        // ==========================================
        // TELEMETRY BEFORE START
        // ==========================================

        telemetry.addLine("================================");
        telemetry.addLine(" APRILTAG CENTER + DISTANCE");
        telemetry.addLine("================================");

        telemetry.addData(
                "Target Tag",
                TARGET_TAG_ID
        );

        telemetry.addData(
                "Target Distance",
                "%.1f inches",
                TARGET_DISTANCE_INCH
        );

        telemetry.addData(
                "Distance Offset",
                "%.1f inches",
                DISTANCE_OFFSET_INCH
        );

        telemetry.update();

        waitForStart();

        // ==========================================
        // MAIN LOOP
        // ==========================================

        while (opModeIsActive()) {

            LLResult result =
                    limelight.getLatestResult();

            // ==========================================
            // CHECK RESULT
            // ==========================================

            if (result == null || !result.isValid()) {

                tr.stopDrive();

                telemetry.addLine(
                        "Limelight: NO VALID RESULT"
                );

                telemetry.update();

                sleep(20);
                continue;
            }

            // ==========================================
            // GET APRILTAGS
            // ==========================================

            List<LLResultTypes.FiducialResult> tags =
                    result.getFiducialResults();

            LLResultTypes.FiducialResult targetTag = null;

            // ==========================================
            // FIND TAG ID 30
            // ==========================================

            for (LLResultTypes.FiducialResult tag : tags) {

                if (tag.getFiducialId() == TARGET_TAG_ID) {

                    targetTag = tag;
                    break;
                }
            }

            // ==========================================
            // TAG NOT FOUND
            // ==========================================

            if (targetTag == null) {

                tr.stopDrive();

                telemetry.addLine(
                        "AprilTag 30 NOT DETECTED"
                );

                telemetry.addLine(
                        "Robot STOPPED"
                );

                telemetry.update();

                sleep(20);
                continue;
            }

            // ==========================================
            // GET CAMERA SPACE POSE
            // ==========================================

            Pose3D targetPose =
                    targetTag.getTargetPoseCameraSpace();

            if (targetPose == null) {

                tr.stopDrive();

                telemetry.addLine(
                        "Target pose unavailable"
                );

                telemetry.update();

                sleep(20);
                continue;
            }

            // ==========================================
            // GET X Y Z
            // ==========================================

            double x =
                    targetPose.getPosition().x;

            double y =
                    targetPose.getPosition().y;

            double z =
                    targetPose.getPosition().z;

            // ==========================================
            // CALCULATE ANGLES
            // ==========================================

            double horizontalAngle =
                    Math.toDegrees(
                            Math.atan2(x, z)
                    );

            double verticalAngle =
                    Math.toDegrees(
                            Math.atan2(y, z)
                    );

            // ==========================================
            // DISTANCE
            // ==========================================

            // Z is in meters
            double rawDistanceInches =
                    Math.abs(z) * 39.3701;

            // Apply calibration
            double calibratedDistanceInches =
                    rawDistanceInches
                            - DISTANCE_OFFSET_INCH;

            // ==========================================
            // DISTANCE ERROR
            // ==========================================

            double distanceError =
                    calibratedDistanceInches
                            - TARGET_DISTANCE_INCH;

            // ==========================================
            // TELEMETRY
            // ==========================================

            telemetry.addLine(
                    "================================"
            );

            telemetry.addLine(
                    "       APRILTAG ALIGNMENT"
            );

            telemetry.addLine(
                    "================================"
            );

            telemetry.addData(
                    "Tag ID",
                    targetTag.getFiducialId()
            );

            telemetry.addLine(
                    "--------------------------------"
            );

            telemetry.addData(
                    "Horizontal Angle",
                    "%.2f degrees",
                    horizontalAngle
            );

            telemetry.addData(
                    "Vertical Angle",
                    "%.2f degrees",
                    verticalAngle
            );

            telemetry.addLine(
                    "--------------------------------"
            );

            telemetry.addData(
                    "X",
                    "%.3f m",
                    x
            );

            telemetry.addData(
                    "Y",
                    "%.3f m",
                    y
            );

            telemetry.addData(
                    "Z",
                    "%.3f m",
                    z
            );

            telemetry.addLine(
                    "--------------------------------"
            );

            telemetry.addData(
                    "Raw Distance",
                    "%.2f inches",
                    rawDistanceInches
            );

            telemetry.addData(
                    "Calibrated Distance",
                    "%.2f inches",
                    calibratedDistanceInches
            );

            telemetry.addData(
                    "Distance Error",
                    "%.2f inches",
                    distanceError
            );

            telemetry.addLine(
                    "--------------------------------"
            );
            // ==========================================
// STATE MACHINE
// ==========================================
//
// STATE 0 = CENTER
// STATE 1 = FORWARD / DISTANCE
// STATE 2 = DONE
//
// ==========================================

            switch (STATE_MANAGE) {

                // ==========================================
                // STATE 0
                // CENTER APRILTAG
                // ==========================================

                case 0:

                    telemetry.addLine(
                            "STATE: CENTERING"
                    );

                    if (horizontalAngle > ANGLE_TOLERANCE) {

                        // Tag is on right
                        tr.driveRight(0.2);

                        telemetry.addLine(
                                "→ STRAFING RIGHT"
                        );

                    } else if (horizontalAngle < -ANGLE_TOLERANCE) {

                        // Tag is on left
                        tr.driveLeft(0.2);

                        telemetry.addLine(
                                "← STRAFING LEFT"
                        );

                    } else {

                        // ==================================
                        // CENTERED
                        // STOP STRAFE
                        // ==================================

                        tr.stopDrive();

                        telemetry.addLine(
                                "✓ CENTERED"
                        );

                        // Move to forward state
                        STATE_MANAGE = 1;

                        telemetry.addLine(
                                "→ SWITCHING TO FORWARD"
                        );
                    }

                    break;


                // ==========================================
                // STATE 1
                // MOVE TO TARGET DISTANCE
                // ==========================================

                case 1:

                    telemetry.addLine(
                            "STATE: FORWARD"
                    );

                    // ======================================
                    // TOO FAR
                    // ======================================

                    if (calibratedDistanceInches >
                            TARGET_DISTANCE_INCH + DISTANCE_TOLERANCE) {

                        tr.drive(0.3);

                        telemetry.addLine(
                                "↑ MOVING FORWARD"
                        );

                    }

                    // ======================================
                    // TOO CLOSE
                    // ======================================

                    else if (calibratedDistanceInches <
                            TARGET_DISTANCE_INCH - DISTANCE_TOLERANCE) {

                        tr.drive(-0.3);

                        telemetry.addLine(
                                "↓ MOVING BACKWARD"
                        );

                    }

                    // ======================================
                    // TARGET DISTANCE REACHED
                    // ======================================

                    else {

                        tr.stopDrive();

                        telemetry.addLine(
                                "✓ 30 INCHES REACHED"
                        );

                        // Move to DONE state
                        STATE_MANAGE = 2;

                        telemetry.addLine(
                                "→ PROCESS COMPLETE"
                        );
                    }

                    break;


                // ==========================================
                // STATE 2
                // DONE
                // ==========================================

                case 2:

                    tr.stopDrive();

                    telemetry.addLine(
                            "STATE: DONE"
                    );

                    telemetry.addLine(
                            "✓ CENTERED"
                    );

                    telemetry.addLine(
                            "✓ 30 INCHES"
                    );

                    telemetry.addLine(
                            "✓ ROBOT STOPPED"
                    );

                    break;
            }
        }

        // ==========================================
        // STOP
        // ==========================================

        tr.stopDrive();

        limelight.stop();
    }
}