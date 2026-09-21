
package org.firstinspires.ftc.teamcode;
import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class AutoPath extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(0, 0, 90);
    private final Pose path1 = poseFactory.of(50, 50, 180);
    private final Pose path2start = poseFactory.of(50, 50, 90);
    private final Pose path2=poseFactory.of(50,70,90);

    private Intake_Balls ib;
    private servo_d s;
    double tolerance = 1.0;
    private boolean intakeTriggered = false;

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2())
//                waitMs(2000),
//                instant(()-> ib.in(0.6)),
//                waitMs(2000),
//                parallel(instant(()->ib.stop1()),instant(()->s.setPB())),
//                waitMs(2000),
//                instant(()-> ib.out(0.6)),
//                waitMs(2000),
//                instant(()->ib.stop1())


        );
    }

//    private Command waitMilliseconds(long milliseconds) {
//
//        final long[] startTime = {0};
//
//        return Command.build()
//
//                .setStart(() -> {
//                    startTime[0] = System.currentTimeMillis();
//                })
//
//                .setDone(() -> {
//                    return System.currentTimeMillis() - startTime[0] >= milliseconds;
//                });
//    }
    @Override
    public void runOpMode() {
        ib=new Intake_Balls(hardwareMap);
        s=new servo_d(hardwareMap);

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

//
//            double distanceToPoint = Math.hypot(follower.pose().x() - 75.0, follower.pose().y() - 75.0);
//
//            if (!intakeTriggered && distanceToPoint <= 1.5) {
//                intakeTriggered = true; // Prevents re-triggering continuously
//
//                // Schedule non-blocking intake sequence using Ivy Commands
//                schedule(sequential(
//                        instant(() -> ib.in(0.6)),
//                        waitMs(2000),
//                        instant(() -> ib.stop1())
//                ));
//            }
//            if (follower.isBusy() && follower.distanceToEndpoint() <= tolerance) {
//                follower.stop(); // Tells Pedro Pathing to stop pathing immediately
//                Scheduler.reset();         // Clears running Ivy command
//            }

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {

        return line(start, path1).linear(start, path1);
    }
    public Path path2() {

        return line(path2start, path2).linear(path2start, path2);
    }
}

