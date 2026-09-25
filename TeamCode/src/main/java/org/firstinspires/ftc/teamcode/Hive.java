package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

@TeleOp(
        name = "BIOBUZZ - HIVE Distance State",
        group = "Limelight"
)
public class Hive extends LinearOpMode {

    // =========================================================
    // LIMELIGHT
    // =========================================================

    private Limelight3A limelight;
    private test_robot tr;


    // =========================================================
    // TARGET SETTINGS
    // =========================================================

    // Desired distance from AprilTag
    private static final double TARGET_DISTANCE_INCH = 24.0;

    // Allowed error around 24 inches
    private static final double TOLERANCE_INCH = 0.5;


    // =========================================================
    // DISTANCE CALIBRATION
    // =========================================================

    /*
     * Your current test:
     *
     * Limelight says = 24"
     * Actual distance = 19"
     *
     * Difference = 5"
     *
     * Therefore:
     *
     * Calibrated Distance = Raw Distance - 5"
     *
     * Change this value after doing more calibration tests.
     */

    private static final double DISTANCE_OFFSET_INCH = 5.0;


    // =========================================================
    // DRIVE STATE
    // =========================================================

    enum DriveState {

        FORWARD,
        STOP,
        REVERSE,
        NO_TAG
    }


    // =========================================================
    // MAIN OP MODE
    // =========================================================

    @Override
    public void runOpMode() {


        // =====================================================
        // INITIALIZE LIMELIGHT
        // =====================================================

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );
        tr=new test_robot(hardwareMap);


        // Select AprilTag pipeline
        limelight.pipelineSwitch(0);


        // Start Limelight
        limelight.start();


        // =====================================================
        // INITIAL TELEMETRY
        // =====================================================

        telemetry.addLine(
                "================================"
        );

        telemetry.addLine(
                "     BIOBUZZ HIVE DISTANCE"
        );

        telemetry.addLine(
                "================================"
        );

        telemetry.addData(
                "Target Distance",
                "%.1f inches",
                TARGET_DISTANCE_INCH
        );

        telemetry.addData(
                "Calibration Offset",
                "%.1f inches",
                DISTANCE_OFFSET_INCH
        );

        telemetry.addLine();

        telemetry.addLine(
                "Limelight Ready"
        );

        telemetry.update();


        // =====================================================
        // WAIT FOR START
        // =====================================================

        waitForStart();


        // =====================================================
        // MAIN LOOP
        // =====================================================

        while (opModeIsActive()) {


            // -------------------------------------------------
            // GET LIMELIGHT RESULT
            // -------------------------------------------------

            LLResult result =
                    limelight.getLatestResult();


            telemetry.clearAll();


            telemetry.addLine(
                    "================================"
            );

            telemetry.addLine(
                    "       BIOBUZZ HIVE TEST"
            );

            telemetry.addLine(
                    "================================"
            );


            // -------------------------------------------------
            // CHECK LIMELIGHT RESULT
            // -------------------------------------------------

            if (result != null && result.isValid()) {


                // Get detected AprilTags
                List<LLResultTypes.FiducialResult> tags =
                        result.getFiducialResults();


                // -------------------------------------------------
                // CHECK FOR APRILTAG
                // -------------------------------------------------

                if (tags != null && !tags.isEmpty()) {


                    // -------------------------------------------------
                    // USE FIRST DETECTED TAG
                    // -------------------------------------------------

                    LLResultTypes.FiducialResult tag =
                            tags.get(0);


                    // -------------------------------------------------
                    // TAG ID
                    // -------------------------------------------------

                    int tagID =
                            tag.getFiducialId();


                    // -------------------------------------------------
                    // GET CAMERA SPACE POSE
                    // -------------------------------------------------

                    Pose3D pose =
                            tag.getTargetPoseCameraSpace();


                    // -------------------------------------------------
                    // GET POSITION
                    // -------------------------------------------------

                    Position position =
                            pose.getPosition();


                    double xMeters =
                            position.x;


                    double yMeters =
                            position.y;


                    double zMeters =
                            position.z;


                    // -------------------------------------------------
                    // RAW DISTANCE
                    // -------------------------------------------------

                    /*
                     * Z is the forward/backward distance
                     * between the Limelight camera and
                     * the AprilTag.
                     *
                     * Pose value is in meters.
                     */

                    double rawDistanceInches =
                            Math.abs(zMeters) * 39.3701;


                    // -------------------------------------------------
                    // CALIBRATED DISTANCE
                    // -------------------------------------------------

                    double calibratedDistanceInches =
                            rawDistanceInches
                                    - DISTANCE_OFFSET_INCH;


                    // -------------------------------------------------
                    // DISTANCE ERROR
                    // -------------------------------------------------

                    double distanceError =
                            calibratedDistanceInches
                                    - TARGET_DISTANCE_INCH;


                    // -------------------------------------------------
                    // DETERMINE DRIVE STATE
                    // -------------------------------------------------

                    DriveState driveState;


                    // Robot is too far
                    if (calibratedDistanceInches >
                            TARGET_DISTANCE_INCH
                                    + TOLERANCE_INCH) {


                        driveState =
                                DriveState.FORWARD;


                    }

                    // Robot is too close
                    else if (calibratedDistanceInches <
                            TARGET_DISTANCE_INCH
                                    - TOLERANCE_INCH) {


                        driveState =
                                DriveState.REVERSE;


                    }

                    // Robot is at target distance
                    else {


                        driveState =
                                DriveState.STOP;
                    }

                    if(calibratedDistanceInches>24){
                        tr.drive(0.5);
                    } else if (calibratedDistanceInches==24) {
                        tr.drive(0);
                    } else {
                        tr.drive(0);
                    }
                    // =================================================
                    // TELEMETRY
                    // =================================================

                    telemetry.addData(
                            "AprilTag",
                            "DETECTED"
                    );


                    telemetry.addData(
                            "Tag ID",
                            tagID
                    );


                    telemetry.addLine(
                            "--------------------------------"
                    );


                    // Camera position
                    telemetry.addData(
                            "X",
                            "%.3f m",
                            xMeters
                    );


                    telemetry.addData(
                            "Y",
                            "%.3f m",
                            yMeters
                    );


                    telemetry.addData(
                            "Z",
                            "%.3f m",
                            zMeters
                    );


                    telemetry.addLine(
                            "--------------------------------"
                    );


                    // Raw distance
                    telemetry.addData(
                            "RAW Distance",
                            "%.2f inches",
                            rawDistanceInches
                    );


                    // Calibration
                    telemetry.addData(
                            "Offset",
                            "- %.2f inches",
                            DISTANCE_OFFSET_INCH
                    );


                    // Final calibrated distance
                    telemetry.addData(
                            "CALIBRATED Distance",
                            "%.2f inches",
                            calibratedDistanceInches
                    );


                    telemetry.addData(
                            "TARGET Distance",
                            "%.2f inches",
                            TARGET_DISTANCE_INCH
                    );


                    telemetry.addData(
                            "Distance Error",
                            "%.2f inches",
                            distanceError
                    );


                    telemetry.addLine(
                            "--------------------------------"
                    );


                    // Drive state
                    telemetry.addData(
                            "DRIVE STATE",
                            driveState
                    );


                    telemetry.addLine(
                            "--------------------------------"
                    );


                    telemetry.addLine(
                            "NO MOTOR POWER IS APPLIED"
                    );


                    telemetry.addLine(
                            "Move robot manually"
                    );


                }

                // -------------------------------------------------
                // NO APRILTAG
                // -------------------------------------------------

                else {


                    telemetry.addData(
                            "AprilTag",
                            "NOT DETECTED"
                    );


                    telemetry.addData(
                            "Tag ID",
                            "NONE"
                    );


                    telemetry.addData(
                            "DRIVE STATE",
                            DriveState.NO_TAG
                    );
                }


            }

            // -------------------------------------------------
            // INVALID LIMELIGHT RESULT
            // -------------------------------------------------

            else {


                telemetry.addData(
                        "Limelight",
                        "NO VALID RESULT"
                );


                telemetry.addData(
                        "AprilTag",
                        "NOT DETECTED"
                );


                telemetry.addData(
                        "DRIVE STATE",
                        DriveState.NO_TAG
                );
            }


            // -------------------------------------------------
            // UPDATE TELEMETRY
            // -------------------------------------------------

            telemetry.update();


            // Small loop delay
            sleep(20);
        }


        // =====================================================
        // STOP LIMELIGHT
        // =====================================================

        limelight.stop();
    }
}