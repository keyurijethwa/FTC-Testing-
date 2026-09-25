package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;


@Autonomous(name = "AprilTag24Auto", group = "Autonomous")
public class AutoLimelightTest extends LinearOpMode {

    // =========================================================
    // PEDRO
    // =========================================================

    private Follower follower;

    private final PoseFactory poseFactory =
            PoseFactory.degrees();

    /*
     * Robot's starting field position.
     *
     * This is ONLY the starting pose.
     * No predefined path is used.
     */
    private final Pose start =
            poseFactory.of(0, 0, 90);


    // =========================================================
    // LIMELIGHT
    // =========================================================

    private Limelight3A limelight;

    /*
     * AprilTag that we want to approach.
     */
    private static final int TARGET_TAG_ID = 30;


    // =========================================================
    // DISTANCE
    // =========================================================

    /*
     * Desired distance from AprilTag.
     */
    private static final double TARGET_DISTANCE_INCHES = 24.0;

    /*
     * Robot can stop between 23 and 25 inches.
     */
    private static final double DISTANCE_TOLERANCE_INCHES = 1.0;


    /*
     * IMPORTANT:
     *
     * Set this according to the units reported by your
     * Limelight/SDK Pose3D.
     *
     * If Pose3D is in meters:
     *      39.3701
     *
     * If your telemetry already reports inches:
     *      1.0
     */
    private static final double POSE_TO_INCHES = 1.0;


    // =========================================================
    // STATE
    // =========================================================

    private boolean tagDetected = false;

    private boolean pathCreated = false;

    private boolean finished = false;


    // =========================================================
    // RUN OP MODE
    // =========================================================

    @Override
    public void runOpMode() {

        // =====================================================
        // INITIALIZE PEDRO
        // =====================================================

        follower = Constants.create(hardwareMap);

        follower.setPose(start);

        follower.update();


        // =====================================================
        // INITIALIZE LIMELIGHT
        // =====================================================

        limelight =
                hardwareMap.get(
                        Limelight3A.class,
                        "limelight"
                );

        /*
         * Pipeline 0 = AprilTag pipeline
         */
        limelight.pipelineSwitch(0);

        limelight.start();


        // =====================================================
        // WAITING FOR START
        // =====================================================

        /*
         * IMPORTANT:
         *
         * We continuously check the AprilTag BEFORE
         * the driver presses START.
         *
         * This lets you see:
         *
         * Tag ID
         * X
         * Y
         * Distance
         * Distance to cover
         *
         * while the robot is stationary.
         */

        while (!isStarted() && !isStopRequested()) {

            follower.update();

            showAprilTagInformation();

            telemetry.update();

            sleep(20);
        }


        // =====================================================
        // STOP REQUESTED
        // =====================================================

        if (isStopRequested()) {

            limelight.stop();

            return;
        }


        // =====================================================
        // STARTED
        // =====================================================

        telemetry.addLine("AUTO STARTED");

        telemetry.update();


        // =====================================================
        // MAIN AUTONOMOUS LOOP
        // =====================================================

        while (opModeIsActive()) {

            follower.update();


            // -------------------------------------------------
            // Create path only once
            // -------------------------------------------------

            if (!pathCreated && !finished) {

                createAprilTagPath();
            }


            // -------------------------------------------------
            // Check path finished
            // -------------------------------------------------

            if (!finished) {

                double distance = getAprilTagDistance();

                if (distance > 0) {

                    telemetry.addData(
                            "LIVE TAG DISTANCE",
                            "%.2f in",
                            distance
                    );

                    if (distance <= TARGET_DISTANCE_INCHES + DISTANCE_TOLERANCE_INCHES) {

                        follower.stop();

                        finished = true;

                        telemetry.addLine(
                                "✓ STOPPED AT 24 INCH TARGET"
                        );
                    }
                }
            }


            // -------------------------------------------------
            // Robot telemetry
            // -------------------------------------------------

            telemetry.addData(
                    "Robot X",
                    "%.2f",
                    follower.pose().x()
            );

            telemetry.addData(
                    "Robot Y",
                    "%.2f",
                    follower.pose().y()
            );

            telemetry.addData(
                    "Robot Heading",
                    "%.2f",
                    follower.pose().heading()
            );


            if (finished) {

                telemetry.addData(
                        "STATUS",
                        "STOPPED AT 24 INCHES"
                );

            } else if (pathCreated) {

                telemetry.addData(
                        "STATUS",
                        "MOVING TO APRILTAG"
                );

            } else {

                telemetry.addData(
                        "STATUS",
                        "SEARCHING FOR APRILTAG"
                );
            }


            telemetry.update();
        }


        // =====================================================
        // CLEANUP
        // =====================================================

        limelight.stop();
    }


    // =========================================================
    // SHOW APRILTAG INFORMATION
    // =========================================================

    private void showAprilTagInformation() {

        LLResult result =
                limelight.getLatestResult();


        // -----------------------------------------------------
        // No result
        // -----------------------------------------------------

        if (result == null) {

            tagDetected = false;

            telemetry.addLine(
                    "Limelight: No result"
            );

            return;
        }


        // -----------------------------------------------------
        // Get all tags
        // -----------------------------------------------------

        List<LLResultTypes.FiducialResult> fiducials =
                result.getFiducialResults();


        if (fiducials == null || fiducials.isEmpty()) {

            tagDetected = false;

            telemetry.addLine(
                    "AprilTag: NOT DETECTED"
            );

            return;
        }


        // -----------------------------------------------------
        // Find target tag
        // -----------------------------------------------------

        LLResultTypes.FiducialResult targetTag = null;


        for (
                LLResultTypes.FiducialResult fiducial
                : fiducials
        ) {

            int id =
                    fiducial.getFiducialId();


            if (id == TARGET_TAG_ID) {

                targetTag = fiducial;

                break;
            }
        }


        // -----------------------------------------------------
        // Target tag not found
        // -----------------------------------------------------

        if (targetTag == null) {

            tagDetected = false;

            telemetry.addData(
                    "Target Tag",
                    TARGET_TAG_ID
            );

            telemetry.addLine(
                    "Target AprilTag NOT FOUND"
            );

            return;
        }


        tagDetected = true;


        // =====================================================
        // GET TAG POSE
        // =====================================================

        Pose3D tagPose =
                targetTag.getTargetPoseRobotSpace();


        /*
         * FTC coordinate system:
         *
         * X = right/left
         * Y = forward from camera
         * Z = up
         *
         * So if the tag is directly in front:
         *
         * X ≈ 0
         * Y = distance
         */

        double rawX =
                tagPose.getPosition().x;

        double rawY =
                -tagPose.getPosition().y;


        // =====================================================
        // CONVERT TO INCHES
        // =====================================================

        double tagX =
                rawX * POSE_TO_INCHES;

        double tagY =
                -tagPose.getPosition().y
                        * POSE_TO_INCHES;


        /*
         * Forward distance.
         */
        double currentDistance =
                tagY;


        /*
         * How much robot needs to move.
         */
        double distanceToCover =
                currentDistance
                        - TARGET_DISTANCE_INCHES;


        // =====================================================
        // TELEMETRY
        // =====================================================

        telemetry.addLine(
                "===== APRILTAG ====="
        );

        telemetry.addData(
                "Tag Detected",
                "YES"
        );

        telemetry.addData(
                "Tag ID",
                targetTag.getFiducialId()
        );

        telemetry.addData(
                "Tag X",
                "%.2f in",
                tagX
        );

        telemetry.addData(
                "Tag Y",
                "%.2f in",
                tagY
        );

        telemetry.addData(
                "Current Distance",
                "%.2f in",
                currentDistance
        );

        telemetry.addData(
                "Target Distance",
                "%.2f in",
                TARGET_DISTANCE_INCHES
        );

        telemetry.addData(
                "Distance To Cover",
                "%.2f in",
                distanceToCover
        );


        if (
                Math.abs(distanceToCover)
                        <= DISTANCE_TOLERANCE_INCHES
        ) {

            telemetry.addLine(
                    "ALREADY AT 24 INCHES"
            );

        } else if (distanceToCover > 0) {

            telemetry.addData(
                    "Direction",
                    "FORWARD"
            );

        } else {

            telemetry.addData(
                    "Direction",
                    "BACKWARD"
            );
        }
    }


    // =========================================================
    // CREATE APRILTAG PATH
    // =========================================================
    private double getAprilTagDistance() {

        LLResult result = limelight.getLatestResult();

        if (result == null) {
            return -1;
        }

        List<LLResultTypes.FiducialResult> fiducials =
                result.getFiducialResults();

        if (fiducials == null || fiducials.isEmpty()) {
            return -1;
        }

        for (LLResultTypes.FiducialResult fiducial : fiducials) {

            if (fiducial.getFiducialId() == TARGET_TAG_ID) {

                Pose3D tagPose =
                        fiducial.getTargetPoseRobotSpace();

                // Your Limelight is giving negative Y
                // when the tag is physically in front.
                double tagY =
                        -tagPose.getPosition().y
                                * POSE_TO_INCHES;

                return tagY;
            }
        }

        return -1;
    }
    private void createAprilTagPath() {

        LLResult result =
                limelight.getLatestResult();


        if (result == null) {

            return;
        }


        List<LLResultTypes.FiducialResult> fiducials =
                result.getFiducialResults();


        if (fiducials == null || fiducials.isEmpty()) {

            return;
        }


        // =====================================================
        // FIND TARGET TAG
        // =====================================================

        LLResultTypes.FiducialResult targetTag = null;


        for (
                LLResultTypes.FiducialResult fiducial
                : fiducials
        ) {

            if (
                    fiducial.getFiducialId()
                            == TARGET_TAG_ID
            ) {

                targetTag = fiducial;

                break;
            }
        }


        if (targetTag == null) {

            return;
        }


        // =====================================================
        // GET TAG POSITION
        // =====================================================

        Pose3D tagPose =
                targetTag.getTargetPoseRobotSpace();


        double tagY =
                -tagPose.getPosition().y
                        * POSE_TO_INCHES;


        // =====================================================
        // CALCULATE DISTANCE
        // =====================================================

        double distanceToCover =
                tagY
                        - TARGET_DISTANCE_INCHES;


        // =====================================================
        // ALREADY AT TARGET
        // =====================================================

        if (
                Math.abs(distanceToCover)
                        <= DISTANCE_TOLERANCE_INCHES
        ) {

            finished = true;

            telemetry.addLine(
                    "Already at 24 inches"
            );

            return;
        }


        // =====================================================
// CURRENT PEDRO POSE
// =====================================================

        Pose currentPose = follower.pose();

        double currentX = currentPose.x();
        double currentY = currentPose.y();
        double currentHeading = currentPose.heading();


// =====================================================
// MOVE STRAIGHT FORWARD
// =====================================================

        double headingRadians =
                Math.toRadians(currentHeading);

// Pedro forward direction
        double forwardX =
                -Math.sin(headingRadians);

        double forwardY =
                Math.cos(headingRadians);

// Move only in robot's forward direction
        double targetX =
                currentX + forwardX * distanceToCover;

        double targetY =
                currentY + forwardY * distanceToCover;


// =====================================================
// CREATE TARGET POSE
// =====================================================

        Pose targetPose =
                poseFactory.of(
                        targetX,
                        targetY,
                        currentHeading
                );


// =====================================================
// CREATE STRAIGHT PATH
// =====================================================

        Path approachPath =
                line(
                        currentPose,
                        targetPose
                ).linear(
                        currentPose,
                        targetPose
                );

        follower.follow(approachPath);

        pathCreated = true;

        telemetry.addLine(
                "PEDRO PATH CREATED"
        );

        telemetry.addData(
                "Move Distance",
                "%.2f in",
                distanceToCover
        );

        telemetry.addData(
                "Target X",
                "%.2f",
                targetX
        );

        telemetry.addData(
                "Target Y",
                "%.2f",
                targetY
        );
    }
}