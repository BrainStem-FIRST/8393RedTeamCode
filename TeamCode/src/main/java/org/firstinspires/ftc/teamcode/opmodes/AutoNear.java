package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.ColorSensorBall;
import org.firstinspires.ftc.teamcode.utils.NullGamepadTracker;

@Config
@Autonomous(name="Auto near")
public class AutoNear extends LinearOpMode {
    public static class Params {
        public double startX = 24.2, startY = 128.2, startA = -90;
        public double obeliskReadX = 42, obeliskReadY = 96.3, obeliskReadA = -80;
        public double shootX = 52.2, shootY = 87, shootA = -45;
        public double preCollectX = 41, collectX = 25, collectA = -180;
        public double collect1Y = 92, collect2Y = 68;
        public double preCollect1TX = 53, collect1TY = 53;
        public double collectDrivePower = 0.085, collectHeadingCorrectAmp = 0.3, collectStrafeCorrectAmp = -0.05;
        public double headingError = 7;
    }
    public static Params params = new Params();
    private Robot robot;
    private Pose startPose, obeliskReadPose, shootPose, preCollect1Pose, collect1Pose, preCollect2Pose, collect2Pose;
    private Path cameraPath;
    private PathChain shoot1Path, preCollect1Path, shoot2Path, preCollect2Path, shoot3Path;

    private int pathNum;
    private ElapsedTime pathTimer;
    private boolean rotatedYet;
    private PIDFController headingPid, translationalPid;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.params.greenPos = -1;
        telemetry.setMsTransmissionInterval(11);

        startPose = new Pose(params.startX, params.startY, Math.toRadians(params.startA));
        obeliskReadPose = new Pose(params.obeliskReadX, params.obeliskReadY, Math.toRadians(params.obeliskReadA));
        shootPose = new Pose(params.shootX, params.shootY, Math.toRadians(params.shootA));
        preCollect1Pose = new Pose(params.preCollectX, params.collect1Y, Math.toRadians(params.collectA));
        collect1Pose = new Pose(params.collectX, params.collect1Y, Math.toRadians(params.collectA));
        preCollect2Pose = new Pose(params.preCollectX, params.collect2Y, Math.toRadians(params.collectA));
        collect2Pose = new Pose(params.collectX, params.collect2Y, Math.toRadians(params.collectA));

        robot = new Robot(hardwareMap, telemetry, new NullGamepadTracker(), new NullGamepadTracker(), startPose);
        telemetry.addData("ball list", robot.indexer.getLabeledBalls());
        telemetry.addData("intake i", robot.indexer.getIntakeI());
        telemetry.addData("preColelct 1 heading", preCollect1Pose.getHeading());
        robot.indexer.setAutoBallList(1, ColorSensorBall.BallColor.P, ColorSensorBall.BallColor.P, ColorSensorBall.BallColor.G);
        robot.indexer.setAutoRotate(true);
        robot.shooter.setZone(1);
        robot.intake.setIntakeSafely(true);
        robot.transfer.setShootSafely(true);

        cameraPath = new Path(new BezierLine(startPose, obeliskReadPose));
        cameraPath.setLinearHeadingInterpolation(startPose.getHeading(), obeliskReadPose.getHeading());

        shoot1Path = robot.follower.pathBuilder()
                .addPath(new BezierLine(obeliskReadPose, shootPose))
                .setLinearHeadingInterpolation(obeliskReadPose.getHeading(), shootPose.getHeading())
                .build();
        preCollect1Path = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootPose, preCollect1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), preCollect1Pose.getHeading())
                .setHeadingConstraint(Math.toRadians(params.headingError))
                .build();
        shoot2Path = robot.follower.pathBuilder()
                .addPath(new BezierLine(collect1Pose, shootPose))
                .setLinearHeadingInterpolation(collect1Pose.getHeading(), shootPose.getHeading())
                .build();
        preCollect2Path = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootPose, preCollect2Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), preCollect2Pose.getHeading())
                .setHeadingConstraint(Math.toRadians(params.headingError))
                .build();
        shoot3Path = robot.follower.pathBuilder()
                .addPath(new BezierLine(collect2Pose, shootPose))
                .setLinearHeadingInterpolation(collect2Pose.getHeading(), shootPose.getHeading())
                .build();

        headingPid = new PIDFController(robot.follower.constants.coefficientsHeadingPIDF);
        translationalPid = new PIDFController(robot.follower.constants.coefficientsTranslationalPIDF);

        pathNum = 0;
        pathTimer = new ElapsedTime();
        telemetry.addData("ready", "");
        telemetry.addData("ball list", robot.indexer.getLabeledBalls());
        telemetry.addData("intake i", robot.indexer.getIntakeI());
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            updateAuto();
            telemetry.addData("pose", Math.floor(robot.follower.getPose().getX()) + ", " + Math.floor(robot.follower.getPose().getY()) + ", " + Math.floor(robot.follower.getPose().getHeading() * 180 / Math.PI));
            telemetry.addData("path i", pathNum);
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
    private void updateAuto() {
        robot.intake.update();
        robot.indexer.update();
        robot.transfer.update();
        robot.shooter.update();
        robot.light.update();
        updatePedroAuto();
    }
    private void updatePedroAuto() {
        switch(pathNum) {
            case 0:
                updatePath(cameraPath);
                break;
            case 1:
                if(!robot.follower.isBusy()) {
                    updatePath(shoot1Path);
                }
                break;
            case 2:
                robot.updateAprilTag();
                if(Robot.params.greenPos != -1 && !rotatedYet) {
                    rotatedYet = true;
                    robot.indexer.rotate(robot.indexer.getAlignIndexerOffset());
                }
                if(!robot.follower.isBusy()) {
                    if(robot.indexer.getNumBalls() == 3)
                        robot.transfer.setShootAll(true);
                    else if(robot.indexer.getNumBalls() == 0) {
                        robot.transfer.setShootAll(false);
                        updatePath(preCollect1Path);
                        robot.intake.setIntake(true);
                    }
                }
                break;
            case 3:
            case 6:
                if(!robot.follower.isBusy()) {
                    updatePath();
                    robot.follower.startTeleopDrive();
                }
                break;
            case 4:
                if(robot.indexer.getNumBalls() < 3 && robot.follower.getPose().getX() > collect1Pose.getX()) {
                    pidDriveY(collect1Pose);
                }
                else {
                    updatePath(shoot2Path);
                    robot.intake.setIntake(false);
                    robot.indexer.rotate(robot.indexer.getAlignIndexerOffset());
                }
                break;
            case 5:
                if(!robot.follower.isBusy()) {
                    if (robot.indexer.getNumBalls() > 0)
                        robot.transfer.setShootAll(true);
                    if(robot.indexer.getNumBalls() == 0) {
                        robot.transfer.setShootAll(false);
                        updatePath(preCollect2Path);
                        robot.intake.setIntake(true);
                    }
                }
                break;
            case 7:
                if(robot.indexer.getNumBalls() < 3 && robot.follower.getPose().getX() > collect2Pose.getX()) {
                    pidDriveY(collect2Pose);
                }
                else {
                    updatePath(shoot3Path);
                    robot.intake.setIntake(false);
                    robot.indexer.rotate(robot.indexer.getAlignIndexerOffset());
                }

                break;
            case 8:
                if(!robot.follower.isBusy())
                    robot.transfer.setShootAll(robot.indexer.getNumBalls() >= 0);
                break;
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
    private void updatePath() {
        pathNum++;
        pathTimer.reset();
    }
    private void pidDriveY(Pose pose) {
        double strafePower = (pose.getY() - robot.follower.getPose().getY()) * params.collectStrafeCorrectAmp;
        double goalHeading = (pose.getHeading() + Math.PI * 2) % (Math.PI * 2);
        double heading = (robot.follower.getHeading() + Math.PI * 2) % (Math.PI * 2);
        double turnPower = (goalHeading - heading) * params.collectHeadingCorrectAmp;
        telemetry.addData("strafe", strafePower);
        telemetry.addData("turn", turnPower);
        telemetry.addData("goal heading", goalHeading);
        telemetry.addData("cur heading", heading);
        robot.follower.setTeleOpDrive(params.collectDrivePower, strafePower, turnPower, true);
    }

}
