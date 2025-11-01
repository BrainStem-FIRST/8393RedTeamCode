package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;

@Config
@Autonomous(name="Auto")
public class Auto extends LinearOpMode {
    public static class Params {
        public double startX = 72, startY = -5, startA = 0;
        public double shootX = 72, shootY = -5, shootA = 0;
        public double preCollect1X = 10, preCollect1Y = -140, preCollect1A = 90;
        public double collect1X = 5, collect1Y = -140, collect1A = 90;
    }
    public static Params params = new Params();
    private Robot robot;
    private int pathNum;
    private ElapsedTime pathTimer;
    private Pose startPose, shootPose, preCollect1Pose, collect1Pose;
    private Path shoot1Path;
    private PathChain preCollect1Path, collect1Path, shoot2Path;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(11);

        startPose = new Pose(params.startX, params.startY, Math.toRadians(params.startA));
        shootPose = new Pose(params.shootX, params.shootY, Math.toRadians(params.shootA));
        preCollect1Pose = new Pose(params.preCollect1X, params.preCollect1Y, params.preCollect1A);
        collect1Pose = new Pose(params.collect1X, params.collect1Y, params.collect1A);

        pathTimer = new ElapsedTime();
        robot = new Robot(hardwareMap, telemetry);
        robot.intake.setIntakeSafely(true); // want intake to auto stop when indexing to prevent jams
        robot.indexer.setAutoRotate(true);
        pathNum = 0;

        shoot1Path = new Path(new BezierLine(startPose, shootPose));
        shoot1Path.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

        preCollect1Path = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootPose, preCollect1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), preCollect1Pose.getHeading())
                .build();

        collect1Path = robot.follower.pathBuilder()
                .addPath(new BezierLine(preCollect1Pose, collect1Pose))
                .setLinearHeadingInterpolation(preCollect1Pose.getHeading(), collect1Pose.getHeading())
                .build();

        shoot2Path = robot.follower.pathBuilder()
                        .addPath(new BezierLine(collect1Pose, shootPose))
                        .setLinearHeadingInterpolation(collect1Pose.getHeading(), shootPose.getHeading())
                        .build();

        waitForStart();

        while(opModeIsActive()) {
            updateAuto();
            telemetry.update();
        }
    }

    private void updateAuto() {
        robot.updateAuto();
        robot.intake.update();
        robot.indexer.update();
        robot.transfer.update();
        robot.shooter.update();
        updatePedroAuto();

    }
    private void updatePedroAuto() {
        switch(pathNum) {
            case 0:
                updatePath(shoot1Path);
                break;
            case 1:
                if(!robot.follower.isBusy())
                    if(robot.indexer.getNumBalls() > 0)
                        robot.transfer.setShootAll(true);
                    else {
                        robot.transfer.setShootAll(false);
                        updatePath(preCollect1Path);
                        robot.intake.setIntake(true);
                    }
                break;
            case 2:
                if(robot.indexer.getNumBalls() == 3 && !robot.follower.isBusy()) {
                    updatePath(shoot2Path);
                    robot.intake.setIntake(false);
                }
                break;
            case 3:
                if(!robot.follower.isBusy()) {
                    robot.transfer.setShootAll(true);
                }
        }
        robot.follower.update();
    }
    private void updatePath(Path path) {
        robot.follower.followPath(path);
        pathNum++;
        pathTimer.reset();
    }
    private void updatePath(PathChain pathChain) {
        robot.follower.followPath(pathChain);
        pathNum++;
        pathTimer.reset();
    }

}
