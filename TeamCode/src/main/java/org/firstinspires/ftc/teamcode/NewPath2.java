package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.pedro.Constants.foresightConfig;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "NewPath2")
public class NewPath2 extends LinearOpMode {
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(60, 15, 90);

    private final Pose path1=poseFactory.of(60,25,90);

    private final Pose path2=poseFactory.of(20,50,180);
    private final Pose path3=poseFactory.of(72,35,270);
    private final Pose path4=poseFactory.of(72,118,270);
    private final Pose path5=poseFactory.of(48,135,90);
    private final Pose path5Start=poseFactory.of(72,118,90);
//    private final Pose path5Control=poseFactory.of(50,120,90);
    private final Pose path6=poseFactory.of(72,118,270);
//    private final Pose path6Control=poseFactory.of(50,120,270);
    private final Pose path6Start=poseFactory.of(48,135,270);
    private final Pose path7=poseFactory.of(60,15,90);
    private Intake_Balls ib;
    private ShootBalls sb;
    private servo_d s;


    public Command autoRoutine() {
        return sequential(
                follow(follower,path1()),
                instant(()->s.setPY()),
                parallel(instant(()->ib.out(0.85)),instant(()->sb.forward(0.5))),
                waitMs(3000),
                parallel(instant(()->ib.stop1()),instant(()->sb.stop()),instant(()->s.setPX())),
                follow(follower, path2()),
                instant(()->ib.in(0.85)),
                waitMs(2000),
                instant(()-> ib.stop1()),
                follow(follower,path3()),
                follow(follower,path4()),
                instant(()->s.setPY()),
                parallel(instant(()->ib.out(0.85)),instant(()->sb.forward(0.5))),
                waitMs(3000),
                parallel(instant(()->ib.stop1()),instant(()->sb.stop()),instant(()->s.setPX())),
                follow(follower,path5()),
                instant(()->ib.in(0.85)),
                waitMs(2000),
                instant(()-> ib.stop1()),
                follow(follower,path6()),
                instant(()->s.setPY()),
                parallel(instant(()->ib.out(0.85)),instant(()->sb.forward(0.5))),
                waitMs(3000),
                parallel(instant(()->ib.stop1()),instant(()->sb.stop()),instant(()->s.setPX())),
                follow(follower,path7())

        );
    }
    @Override
    public void runOpMode() throws InterruptedException {
        ib=new Intake_Balls(hardwareMap);
        sb=new ShootBalls(hardwareMap);
        s=new servo_d(hardwareMap);

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        s.setPX();
        s.setHalf();
        telemetry.addData("Stopper set",true);
        telemetry.update();
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
        return line(path2,path3).linear(path2,path3);
    }
    public Path path4(){
        return line(path3,path4).linear(path3,path4);
    }
    public Path path5(){
        return line(path5Start,path5).linear(path5Start,path5);
    }
    public Path path6(){
        return line(path6Start,path6).linear(path6Start,path6);
    }
    public Path path7(){
        return line(path6,path7).linear(path6,path7);
    }
}
