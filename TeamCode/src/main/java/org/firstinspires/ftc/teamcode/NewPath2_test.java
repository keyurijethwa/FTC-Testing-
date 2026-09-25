package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "NewPath2_test")
public class NewPath2_test extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // =========================================================
    // POSES
    // =========================================================

    private final Pose start =
            poseFactory.of(60, 15, 90);

    private final Pose path1 =
            poseFactory.of(60, 35, 90);

    private final Pose path2 =
            poseFactory.of(20, 35, 180);

    private final Pose path3 =
            poseFactory.of(72, 35, 270);

    private final Pose path4 =
            poseFactory.of(72, 110, 270);

    private final Pose path5Start =
            poseFactory.of(72, 110, 90);

    private final Pose path5 =
            poseFactory.of(40, 130, 90);

    private final Pose path6Start =
            poseFactory.of(40, 130, 270);

    private final Pose path6 =
            poseFactory.of(72, 110, 270);

    private final Pose path7 =
            poseFactory.of(60, 15, 90);


    private Intake_Balls ib;


    // =========================================================
    // TRIGGER FLAGS
    // =========================================================

    private boolean path1ActionDone = false;
    private boolean path2ActionDone = false;
    private boolean path3ActionDone = false;
    private boolean path4ActionDone = false;
    private boolean path5ActionDone = false;
    private boolean path6ActionDone = false;
    private boolean path7ActionDone = false;


    // =========================================================
    // WHEN SHOULD SECOND ACTIVITY START?
    //
    // Change these values.
    //
    // 0.25 = 25%
    // 0.50 = 50%
    // 0.75 = 75%
    // =========================================================

    private final double PATH1_TRIGGER = 0.30;
    private final double PATH2_TRIGGER = 0.30;
    private final double PATH3_TRIGGER = 0.30;
    private final double PATH4_TRIGGER = 0.30;
    private final double PATH5_TRIGGER = 0.30;
    private final double PATH6_TRIGGER = 0.30;
    private final double PATH7_TRIGGER = 0.30;


    // =========================================================
    // AUTO ROUTINE
    // =========================================================

    public Command autoRoutine() {

        return sequential(

                // ---------------------------------------------
                // PATH 1
                // ---------------------------------------------

                follow(follower, path1()),

                // This happens AFTER path 1 is complete
                instant(() -> ib.stop1()),

                waitMs(1000),


                // ---------------------------------------------
                // PATH 2
                // ---------------------------------------------

                follow(follower, path2()),

                instant(() -> ib.stop1()),

                waitMs(1000),


                // ---------------------------------------------
                // PATH 3
                // ---------------------------------------------

                follow(follower, path3()),


                // ---------------------------------------------
                // PATH 4
                // ---------------------------------------------

                follow(follower, path4()),

                instant(() -> ib.stop1()),

                waitMs(1000),


                // ---------------------------------------------
                // PATH 5
                // ---------------------------------------------

                follow(follower, path5()),

                instant(() -> ib.stop1()),


                // ---------------------------------------------
                // PATH 6
                // ---------------------------------------------

                follow(follower, path6()),

                instant(() -> ib.stop1()),


                // ---------------------------------------------
                // PATH 7
                // ---------------------------------------------

                follow(follower, path7()),

                instant(() -> ib.stop1())
        );
    }


    // =========================================================
    // CALCULATE PATH PROGRESS
    //
    // Returns:
    //
    // 0.0 = path start
    // 0.5 = 50%
    // 1.0 = path end
    //
    // This is used because getCurrentTValue() is not
    // available in your Pedro Pathing 3.0.1 Follower.
    // =========================================================

    private double getPathProgress(Pose pathStart, Pose pathEnd) {

        double totalDistance = Math.hypot(
                pathEnd.x() - pathStart.x(),
                pathEnd.y() - pathStart.y()
        );

        double travelledDistance = Math.hypot(
                follower.pose().x() - pathStart.x(),
                follower.pose().y() - pathStart.y()
        );

        if (totalDistance <= 0) {
            return 1.0;
        }

        double progress = travelledDistance / totalDistance;

        // Keep the value between 0 and 1
        return Math.max(0.0, Math.min(1.0, progress));
    }


    // =========================================================
    // CHECK ACTIVITIES WHILE PATH IS RUNNING
    // =========================================================

    private void checkPathActions() {

        if (follower.currentPath() == null) {
            return;
        }


        int currentPath = follower.pathIndex();


        // =====================================================
        // PATH 1
        // =====================================================

        if (currentPath == 0 && !path1ActionDone) {

            double progress =
                    getPathProgress(start, path1);

            if (progress >= PATH1_TRIGGER) {

                path1ActionDone = true;

                // SECOND ACTIVITY
                ib.out(0.7);

                telemetry.addData(
                        "PATH 1 ACTION",
                        "STARTED"
                );
            }
        }


        // =====================================================
        // PATH 2
        // =====================================================

        if (currentPath == 1 && !path2ActionDone) {

            double progress =
                    getPathProgress(path1, path2);

            if (progress >= PATH2_TRIGGER) {

                path2ActionDone = true;

                // SECOND ACTIVITY
                ib.in(0.7);

                telemetry.addData(
                        "PATH 2 ACTION",
                        "STARTED"
                );
            }
        }


        // =====================================================
        // PATH 3
        // =====================================================

        if (currentPath == 2 && !path3ActionDone) {

            double progress =
                    getPathProgress(path2, path3);

            if (progress >= PATH3_TRIGGER) {

                path3ActionDone = true;

                // SECOND ACTIVITY
                ib.out(0.7);

                telemetry.addData(
                        "PATH 3 ACTION",
                        "STARTED"
                );
            }
        }


        // =====================================================
        // PATH 4
        // =====================================================

        if (currentPath == 3 && !path4ActionDone) {

            double progress =
                    getPathProgress(path3, path4);

            if (progress >= PATH4_TRIGGER) {

                path4ActionDone = true;

                // SECOND ACTIVITY
                ib.in(0.7);

                telemetry.addData(
                        "PATH 4 ACTION",
                        "STARTED"
                );
            }
        }


        // =====================================================
        // PATH 5
        // =====================================================

        if (currentPath == 4 && !path5ActionDone) {

            double progress =
                    getPathProgress(path5Start, path5);

            if (progress >= PATH5_TRIGGER) {

                path5ActionDone = true;

                // SECOND ACTIVITY
                ib.out(0.7);

                telemetry.addData(
                        "PATH 5 ACTION",
                        "STARTED"
                );
            }
        }


        // =====================================================
        // PATH 6
        // =====================================================

        if (currentPath == 5 && !path6ActionDone) {

            double progress =
                    getPathProgress(path6Start, path6);

            if (progress >= PATH6_TRIGGER) {

                path6ActionDone = true;

                // SECOND ACTIVITY
                ib.out(0.7);

                telemetry.addData(
                        "PATH 6 ACTION",
                        "STARTED"
                );
            }
        }


        // =====================================================
        // PATH 7
        // =====================================================

        if (currentPath == 6 && !path7ActionDone) {

            double progress =
                    getPathProgress(path6, path7);

            if (progress >= PATH7_TRIGGER) {

                path7ActionDone = true;

                // SECOND ACTIVITY
                ib.in(0.7);

                telemetry.addData(
                        "PATH 7 ACTION",
                        "STARTED"
                );
            }
        }
    }


    // =========================================================
    // RUN OP MODE
    // =========================================================

    @Override
    public void runOpMode() throws InterruptedException {

        ib = new Intake_Balls(hardwareMap);

        Scheduler.reset();

        follower = Constants.create(hardwareMap);

        follower.setPose(start);

        follower.update();


        waitForStart();


        if (isStopRequested()) {
            return;
        }


        schedule(autoRoutine());


        while (opModeIsActive()) {

            // Update Pedro
            follower.update();

            // Execute Ivy
            Scheduler.execute();

            // Check progress-based activities
            checkPathActions();


            // =================================================
            // TELEMETRY
            // =================================================

            telemetry.addData(
                    "X",
                    "%.2f",
                    follower.pose().x()
            );

            telemetry.addData(
                    "Y",
                    "%.2f",
                    follower.pose().y()
            );

            telemetry.addData(
                    "Heading",
                    "%.2f",
                    follower.pose().heading()
            );


            if (follower.currentPath() != null) {

                int currentPath =
                        follower.pathIndex();

                telemetry.addData(
                        "Current Path",
                        currentPath
                );

                telemetry.addData(
                        "Distance Remaining",
                        "%.2f",
                        follower.distanceToEndpoint()
                );


                // Show calculated progress
                double progress = 0;


                if (currentPath == 0) {
                    progress =
                            getPathProgress(start, path1);
                }

                else if (currentPath == 1) {
                    progress =
                            getPathProgress(path1, path2);
                }

                else if (currentPath == 2) {
                    progress =
                            getPathProgress(path2, path3);
                }

                else if (currentPath == 3) {
                    progress =
                            getPathProgress(path3, path4);
                }

                else if (currentPath == 4) {
                    progress =
                            getPathProgress(path5Start, path5);
                }

                else if (currentPath == 5) {
                    progress =
                            getPathProgress(path6Start, path6);
                }

                else if (currentPath == 6) {
                    progress =
                            getPathProgress(path6, path7);
                }


                telemetry.addData(
                        "Path Progress",
                        "%.1f%%",
                        progress * 100
                );
            }


            telemetry.update();
        }
    }


    // =========================================================
    // PATH 1
    // =========================================================

    public Path path1() {

        return line(start, path1)
                .linear(start, path1);
    }


    // =========================================================
    // PATH 2
    // =========================================================

    public Path path2() {

        return line(path1, path2)
                .linear(path1, path2);
    }


    // =========================================================
    // PATH 3
    // =========================================================

    public Path path3() {

        return line(path2, path3)
                .linear(path2, path3);
    }


    // =========================================================
    // PATH 4
    // =========================================================

    public Path path4() {

        return line(path3, path4)
                .linear(path3, path4);
    }


    // =========================================================
    // PATH 5
    // =========================================================

    public Path path5() {

        return line(path5Start, path5)
                .linear(path5Start, path5);
    }


    // =========================================================
    // PATH 6
    // =========================================================

    public Path path6() {

        return line(path6Start, path6)
                .linear(path6Start, path6);
    }


    // =========================================================
    // PATH 7
    // =========================================================

    public Path path7() {

        return line(path6, path7)
                .linear(path6, path7);
    }
}