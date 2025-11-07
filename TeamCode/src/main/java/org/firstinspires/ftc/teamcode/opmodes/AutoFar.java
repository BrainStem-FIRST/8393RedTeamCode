package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.ColorSensorBall.BallColor;

@Config
@Autonomous(name="AutoFar")
public class AutoFar extends LinearOpMode {
    public static class Params {
        public double startX = 56.875, startY = 9.75, startA = -90;
        public double shootX = 53.905, shootY = 14.62, shootA = -70.07;
        public double preCollect1X = 18.6, preCollect1Y = 25.1, preCollect1A = -170;
        double pc1x = (shootX + preCollect1X) / 2, pc1y = 30;
        public double collect1X = Robot.params.intakeToWheelCenter + 0.3, collect1Y = Robot.params.width/2 + 0.3, collect1A = -180;
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
        Robot.params.greenPos = -1;
        telemetry.setMsTransmissionInterval(11);

        startPose = new Pose(params.startX, params.startY, Math.toRadians(params.startA));
        shootPose = new Pose(params.shootX, params.shootY, Math.toRadians(params.shootA));
        preCollect1Pose = new Pose(params.preCollect1X, params.preCollect1Y, Math.toRadians(params.preCollect1A));
        collect1Pose = new Pose(params.collect1X, params.collect1Y, Math.toRadians(params.collect1A));

        pathTimer = new ElapsedTime();
        robot = new Robot(hardwareMap, telemetry, startPose);
        robot.intake.setIntakeSafely(true); // want intake to auto stop when indexing to prevent jams
        robot.indexer.setAutoRotate(true);
        robot.shooter.setResting(false);
        robot.indexer.setAutoBallList(1, BallColor.G, BallColor.P, BallColor.P);
        robot.shooter.setZone(2);
        pathNum = 0;

        shoot1Path = new Path(new BezierLine(startPose, shootPose));
        shoot1Path.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

        preCollect1Path = robot.follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(params.pc1x, params.pc1y), preCollect1Pose))
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
            robot.updateAprilTag();
            robot.updateSubsystems();
            updatePedroAuto();
            telemetry.addData("pose", robot.follower.getPose());
            telemetry.addData("path i", pathNum);
            telemetry.addData("timer", pathTimer.seconds());
            telemetry.addData("green pos", Robot.params.greenPos);
            telemetry.addData("follower busy", robot.follower.isBusy());
            telemetry.addData("num balls", robot.indexer.getNumBalls());
            telemetry.addLine();
            telemetry.addData("shooter vel", Math.floor(robot.shooter.getShooterVelocity() * 100)/100);
            telemetry.addData("min shooter vel", robot.shooter.getMinShooterVel());
            telemetry.addLine();
            telemetry.addData("intake i", robot.indexer.getIntakeI());
            telemetry.addData("ballList", robot.indexer.getLabeledBalls());
            telemetry.addLine();
            telemetry.addData("intake state", robot.intake.getIntakeState());
            telemetry.addData("transfer state", robot.transfer.getTransferState());
            telemetry.update();
        }
    }
    private void updatePedroAuto() {
        switch(pathNum) {
            case 0:
                updatePath(shoot1Path);
                break;
            case 1:
                if(!robot.follower.isBusy())
                    if(robot.indexer.getNumBalls() == 3 && Robot.params.greenPos != -1) {
                        robot.indexer.rotate(robot.indexer.getAlignIndexerOffset());
                        robot.transfer.setShootAll(true);
                    }
                    else if(robot.indexer.getNumBalls() == 0) {
                        robot.transfer.setShootAll(false);
                        updatePath(preCollect1Path);
                        robot.intake.setIntake(true);
                    }
                break;
            case 2:
                if(robot.indexer.getNumBalls() == 2 && !robot.follower.isBusy())
                    updatePath(collect1Path);
                break;
            case 3:
                if(robot.indexer.getNumBalls() == 3 && !robot.follower.isBusy()) {
                    robot.intake.setIntake(false);
                    updatePath(shoot2Path);
                }
            case 4:
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
