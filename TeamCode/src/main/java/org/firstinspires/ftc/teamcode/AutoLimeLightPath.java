package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;


@Autonomous(name = "AutoPath22", group = "Autonomous")
public class AutoLimeLightPath extends LinearOpMode {

    // ---------------------------------------------------------
    // PEDRO PATHING
    // ---------------------------------------------------------

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start =
            poseFactory.of(0, 0, 90);

    private final Pose path1Pose =
            poseFactory.of(50, 50, 180);

    private final Pose path2Start =
            poseFactory.of(50, 50, 90);

    private final Pose path2Pose =
            poseFactory.of(50, 70, 90);


    // ---------------------------------------------------------
    // LIMELIGHT
    // ---------------------------------------------------------

    private Limelight3A limelight;

    // Change this to the AprilTag ID you want to approach
    private static final int TARGET_TAG_ID = 30;

    // Desired distance from AprilTag
    private static final double TARGET_DISTANCE_INCHES = 24.0;

    // Robot is considered close enough within this tolerance
    private static final double DISTANCE_TOLERANCE_INCHES = 1.0;


    // ---------------------------------------------------------
    // APRILTAG APPROACH SETTINGS
    // ---------------------------------------------------------

    /*
     * Limelight robot-space:
     *
     * X = left/right
     * Y = forward/backward
     *
     * We want:
     *
     * X = 0
     * Y = 24 inches
     *
     * Therefore the desired robot-relative tag position is:
     *
     * (0, 24)
     */

    private static final double TARGET_X_INCHES = 0.0;
    private static final double TARGET_Y_INCHES = 24.0;


    // ---------------------------------------------------------
    // MAIN AUTONOMOUS ROUTINE
    // ---------------------------------------------------------

    public Command autoRoutine() {

        return sequential(

                // First normal path
                follow(follower, path1()),

                // Second normal path
                follow(follower, path2()),

                // Then approach AprilTag
                approachAprilTag()
        );
    }


    // ---------------------------------------------------------
    // OPMODE
    // ---------------------------------------------------------

    @Override
    public void runOpMode() {

        // -----------------------------------------------------
        // INITIALIZE PEDRO
        // -----------------------------------------------------

        Scheduler.reset();

        follower = Constants.create(hardwareMap);

        follower.setPose(start);

        follower.update();


        // -----------------------------------------------------
        // INITIALIZE LIMELIGHT
        // -----------------------------------------------------

        limelight =
                hardwareMap.get(
                        Limelight3A.class,
                        "limelight"
                );

        // Pipeline 0 must be your AprilTag pipeline
        limelight.pipelineSwitch(0);

        limelight.start();


        // -----------------------------------------------------
        // WAIT
        // -----------------------------------------------------

        telemetry.addLine("Robot Initialized");
        telemetry.addLine("Waiting for Start...");
        telemetry.update();

        waitForStart();


        if (isStopRequested()) {
            limelight.stop();
            return;
        }


        // -----------------------------------------------------
        // START AUTONOMOUS
        // -----------------------------------------------------

        schedule(autoRoutine());


        // -----------------------------------------------------
        // MAIN LOOP
        // -----------------------------------------------------

        while (opModeIsActive()) {

            // Update Pedro
            follower.update();

            // Execute Ivy commands
            Scheduler.execute();


            // -------------------------------------------------
            // PEDRO TELEMETRY
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
                    "Heading",
                    "%.2f",
                    follower.pose().heading()
            );


            if (follower.currentPath() != null) {

                telemetry.addData(
                        "Path Remaining",
                        "%.2f",
                        follower.distanceToEndpoint()
                );

                telemetry.addData(
                        "Path Number",
                        follower.pathIndex()
                );
            }


            // -------------------------------------------------
            // LIMELIGHT TELEMETRY
            // -------------------------------------------------

            LLResult result =
                    limelight.getLatestResult();

            if (result != null) {

                List<LLResultTypes.FiducialResult> fiducials =
                        result.getFiducialResults();

                telemetry.addData(
                        "AprilTags",
                        fiducials.size()
                );

                boolean foundTarget = false;


                for (LLResultTypes.FiducialResult fiducial : fiducials) {

                    int id = fiducial.getFiducialId();

                    telemetry.addData(
                            "Tag",
                            id
                    );


                    // -----------------------------------------
                    // TARGET TAG
                    // -----------------------------------------

                    if (id == TARGET_TAG_ID) {

                        foundTarget = true;

                        Pose3D tagPose =
                                fiducial.getTargetPoseRobotSpace();


                        double tagX =
                                tagPose.getPosition().x;

                        double tagY =
                                tagPose.getPosition().y;

                        double tagZ =
                                tagPose.getPosition().z;


                        telemetry.addData(
                                "TARGET TAG",
                                id
                        );

                        telemetry.addData(
                                "Tag X",
                                "%.2f",
                                tagX
                        );

                        telemetry.addData(
                                "Tag Y",
                                "%.2f",
                                tagY
                        );

                        telemetry.addData(
                                "Tag Z",
                                "%.2f",
                                tagZ
                        );


                        // -------------------------------------
                        // DISTANCE
                        // -------------------------------------

                        /*
                         * Pose3D values from Limelight are
                         * represented in meters.
                         *
                         * Convert to inches.
                         */

                        double xInches =
                                tagX * 39.3701;

                        double yInches =
                                tagY * 39.3701;


                        double distanceInches =
                                Math.hypot(
                                        xInches,
                                        yInches
                                );


                        telemetry.addData(
                                "Tag X Inches",
                                "%.2f",
                                xInches
                        );

                        telemetry.addData(
                                "Tag Y Inches",
                                "%.2f",
                                yInches
                        );

                        telemetry.addData(
                                "Tag Distance",
                                "%.2f in",
                                distanceInches
                        );


                        // -------------------------------------
                        // DISTANCE ERROR
                        // -------------------------------------

                        double distanceError =
                                distanceInches
                                        - TARGET_DISTANCE_INCHES;


                        telemetry.addData(
                                "Distance Error",
                                "%.2f in",
                                distanceError
                        );


                        if (Math.abs(distanceError)
                                <= DISTANCE_TOLERANCE_INCHES) {

                            telemetry.addLine(
                                    "AT 24 INCH TARGET"
                            );
                        }
                    }
                }


                if (!foundTarget) {

                    telemetry.addData(
                            "Target Tag",
                            "NOT FOUND"
                    );
                }

            } else {

                telemetry.addLine(
                        "Limelight Result = NULL"
                );
            }


            telemetry.update();
        }


        // -----------------------------------------------------
        // CLEANUP
        // -----------------------------------------------------

        limelight.stop();
    }


    // =========================================================
    // APRILTAG APPROACH COMMAND
    // =========================================================

    private Command approachAprilTag() {

        return Command.build()

                // ---------------------------------------------
                // START
                // ---------------------------------------------

                .setStart(() -> {

                    telemetry.addLine(
                            "Starting AprilTag Approach"
                    );

                    telemetry.update();
                })


                // ---------------------------------------------
                // COMMAND FINISHED?
                // ---------------------------------------------

                .setDone(() -> {

                    LLResult result =
                            limelight.getLatestResult();

                    if (result == null) {
                        return false;
                    }


                    List<LLResultTypes.FiducialResult> fiducials =
                            result.getFiducialResults();


                    if (fiducials.isEmpty()) {
                        return false;
                    }


                    for (
                            LLResultTypes.FiducialResult fiducial
                            : fiducials
                    ) {

                        if (
                                fiducial.getFiducialId()
                                        == TARGET_TAG_ID
                        ) {

                            Pose3D tagPose =
                                    fiducial
                                            .getTargetPoseRobotSpace();


                            double tagX =
                                    tagPose.getPosition().x
                                            * 39.3701;

                            double tagY =
                                    tagPose.getPosition().y
                                            * 39.3701;


                            double distance =
                                    Math.hypot(
                                            tagX,
                                            tagY
                                    );


                            return Math.abs(
                                    distance
                                            - TARGET_DISTANCE_INCHES
                            ) <= DISTANCE_TOLERANCE_INCHES;
                        }
                    }


                    return false;
                });
    }


    // =========================================================
    // PEDRO PATH 1
    // =========================================================

    public Path path1() {

        return line(
                start,
                path1Pose
        ).linear(
                start,
                path1Pose
        );
    }


    // =========================================================
    // PEDRO PATH 2
    // =========================================================

    public Path path2() {

        return line(
                path2Start,
                path2Pose
        ).linear(
                path2Start,
                path2Pose
        );
    }
}