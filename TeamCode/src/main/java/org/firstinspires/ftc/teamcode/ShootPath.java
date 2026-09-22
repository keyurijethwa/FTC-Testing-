package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.curve;
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

@Autonomous(name = "Shoot Path", group = "Autonomous")
public class ShootPath extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(60, 15, 90);
    private final Pose path1 = poseFactory.of(60, 44, 90);

    private final  Pose path2 = poseFactory.of(15,15,180);
    private final Pose path3 = poseFactory.of(60,105,270);
    private final Pose point3Control1 = poseFactory.of(11, 80, 0);
    private final Pose path4=poseFactory.of(60,20,270);


    private Intake_Balls ib;
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                instant(()->ib.out(0.5)),
                waitMs(1000),
                instant(()-> ib.stop1()),
                waitMs(2000),
                follow(follower,path2()),
                instant(()-> ib.in(0.5)),
                waitMs(2000),
                instant(()-> ib.stop1()),
                follow(follower,path3()),
                instant(()->ib.out(0.5)),
                waitMs(2000),
                instant(()-> ib.stop1()),
                follow(follower,path4())

        );
    }
    @Override
    public void runOpMode() throws InterruptedException {
        ib=new Intake_Balls(hardwareMap);


        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

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

    public Path path2(){
        return line(path1,path2).linear(path1,path2);
    }
    public Path path3(){
        return curve(path2,point3Control1,path3).linear(path2,path3);
    }
    public Path path4(){
        return line(path3,path4).linear(path3,path4);
    }
}
