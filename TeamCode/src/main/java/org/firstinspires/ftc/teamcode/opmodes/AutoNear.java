package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.RGBLight;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.ColorSensorBall;
import org.firstinspires.ftc.teamcode.utils.NullGamepadTracker;

@Config
@Autonomous(name="Auto near")
public class AutoNear extends LinearOpMode {
    public static class Params {
        public double startX = 63, startY = 137, startA = -90;
        public double endX = 46, endY = 72, endA = -45;
        public double shootX = 57, shootY = 82, shootA = -48;
        public double collectX = 25, collectA = -175;

        public double preCollectX1 = 42, collect1Y = 92;

        public double preCollectX2 = 47, collect2Y = 67;
        public double preCollect2TX = 54, preCollect2TY = 64;

        public double preCollectX3 = 46, collectY3 = 43;
        public double preCollect3TX = 53, preCollect3TY = 39;

        public double leverX = 25, leverY = 80, leverA = -180;

        public double shoot2TX = 54, shoot2TY = 84, shoot2TA = -170;

        public double collectDrivePower = 0.14, collectHeadingCorrectAmp = 0.3, collectStrafeCorrectAmp = -0.05;
        public double aprilTagTimeout = 0.5, aprilTagLightTime = 0.8;
        public double headingError = 7;
        public double translationalKP = 0.06, headingKP = 0.02;
        public double leverWait = 0.2;
    }
    public static Params params = new Params();
    private Robot robot;
    private Pose startPose, shootPose, preCollect1Pose, collect1Pose, leverPose, shoot2TPose, preCollect2Pose, preCollect2TPose, collect2Pose, preCollect3Pose, preCollect3TPose, collect3Pose, endPose;
    private PathChain shootAndCameraPath, preCollect1Path, leverPath, shoot2Path, preCollect2Path, shoot3Path, preCollect3Path, shoot4Path, endPath;

    private int pathNum;
    private ElapsedTime pathTimer;
    private boolean rotatedYet;
    private PIDFController headingPid, translationalPid;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot.params.greenPos = -1;
        telemetry.setMsTransmissionInterval(11);

        headingPid = new PIDFController(new PIDFCoefficients(params.headingKP, 0, 0, 0));
        headingPid.setTargetPosition(0);
        translationalPid = new PIDFController(new PIDFCoefficients(params.translationalKP, 0, 0, 0));
        translationalPid.setTargetPosition(0);

        startPose = new Pose(params.startX, params.startY, Math.toRadians(params.startA));
        shootPose = new Pose(params.shootX, params.shootY, Math.toRadians(params.shootA));
        preCollect1Pose = new Pose(params.preCollectX1, params.collect1Y, Math.toRadians(params.collectA));
        collect1Pose = new Pose(params.collectX, params.collect1Y, Math.toRadians(params.collectA));
        leverPose = new Pose(params.leverX, params.leverY, Math.toRadians(params.leverA));
        shoot2TPose = new Pose(params.shoot2TX, params.shoot2TY, Math.toRadians(params.shoot2TA));
        preCollect2Pose = new Pose(params.preCollectX2, params.collect2Y, Math.toRadians(params.collectA));
        preCollect2TPose = new Pose(params.preCollect2TX, params.preCollect2TY);
        collect2Pose = new Pose(params.collectX, params.collect2Y, Math.toRadians(params.collectA));
        preCollect3Pose = new Pose(params.preCollectX3, params.collectY3, Math.toRadians(params.collectA));
        preCollect3TPose = new Pose(params.preCollect3TX, params.preCollect3TY);
        collect3Pose = new Pose(params.collectX, params.collectY3, Math.toRadians(params.collectA));
        endPose = new Pose(params.endX, params.endY, Math.toRadians(params.endA));

        robot = new Robot(hardwareMap, telemetry, new NullGamepadTracker(), new NullGamepadTracker(), startPose);
        telemetry.addData("ball list", robot.indexer.getLabeledBalls());
        telemetry.addData("intake i", robot.indexer.getIntakeI());
        telemetry.addData("preCollect 1 heading", preCollect1Pose.getHeading());
        robot.indexer.setAutoBallList(1, ColorSensorBall.BallColor.P, ColorSensorBall.BallColor.P, ColorSensorBall.BallColor.G);
        robot.indexer.setAutoRotate(true);
        robot.shooter.setZone(0);
        robot.intake.setIntakeSafely(false);
        robot.transfer.setShootSafely(true);

        shootAndCameraPath = robot.follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        preCollect1Path = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootPose, preCollect1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), preCollect1Pose.getHeading())
                .setHeadingConstraint(Math.toRadians(params.headingError))
                .build();
        leverPath = robot.follower.pathBuilder()
                .addPath(new BezierLine(collect1Pose, leverPose))
                .setLinearHeadingInterpolation(collect1Pose.getHeading(), leverPose.getHeading())
                .build();
        shoot2Path = robot.follower.pathBuilder()
                .addPath(new BezierLine(collect1Pose, shootPose))
                .setLinearHeadingInterpolation(collect1Pose.getHeading(), shootPose.getHeading())
//                .addPath(new BezierLine(collect1Pose, shoot2TPose))
//                .setLinearHeadingInterpolation(collect1Pose.getHeading(), shoot2TPose.getHeading())
//                .addPath(new BezierLine(shoot2TPose, shootPose))
//                .setLinearHeadingInterpolation(shoot2TPose.getHeading(), shootPose.getHeading())
                .build();
        preCollect2Path = robot.follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, preCollect2TPose, preCollect2Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), preCollect2Pose.getHeading())
                .setHeadingConstraint(Math.toRadians(params.headingError))
                .build();
        shoot3Path = robot.follower.pathBuilder()
                .addPath(new BezierLine(collect2Pose, shootPose))
                .setLinearHeadingInterpolation(collect2Pose.getHeading(), shootPose.getHeading())
                .build();
        preCollect3Path = robot.follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, preCollect3TPose, preCollect3Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), preCollect3Pose.getHeading())
                .build();
        shoot4Path = robot.follower.pathBuilder()
                .addPath(new BezierLine(collect3Pose, shootPose))
                .setLinearHeadingInterpolation(collect3Pose.getHeading(), shootPose.getHeading())
                .build();
        endPath = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
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
            robot.updateSubsystems();
            if (updatePedroAuto())
                break;

            telemetry.addData("pose", Math.floor(robot.follower.getPose().getX()) + ", " + Math.floor(robot.follower.getPose().getY()) + ", " + Math.floor(robot.follower.getPose().getHeading() * 180 / Math.PI));
            telemetry.addData("path i & timer", pathNum + ", " + pathTimer.seconds());
//            telemetry.addData("green pos", Robot.params.greenPos);
            telemetry.addData("follower busy", robot.follower.isBusy());
            telemetry.addData("num balls", robot.indexer.getNumBalls());
//            telemetry.addData("rotatedYet", rotatedYet);
            telemetry.addData("indexer state", robot.indexer.getIndexerState());
            telemetry.addData("should shoot", robot.transfer.shouldShootAll());
            telemetry.addData("intake i", robot.indexer.getIntakeI());
            telemetry.addData("ballList", robot.indexer.getLabeledBalls());
            telemetry.addData("shooter power", robot.shooter.getShooterPower());
            telemetry.addData("shooter vel", Math.floor(robot.shooter.getShooterVelocity() * 100)/100);
            telemetry.addData("min shooter vel", robot.shooter.getMinShooterVel());
            telemetry.addData("intake state", robot.intake.getIntakeState());
            telemetry.addData("transfer state", robot.transfer.getTransferState());
            telemetry.update();
        }
    }
    private boolean updatePedroAuto() {
        switch(pathNum) {
            case 0:
                updatePath(shootAndCameraPath);
                break;
            case 1:
                robot.updateAprilTag();
                if(Robot.params.greenPos != -1 && !rotatedYet) {
                    rotatedYet = true;
                    robot.indexer.rotate(robot.indexer.getAlignIndexerOffset());
                    robot.rgbLight.setState(RGBLight.LightState.FLASH, RGBLight.params.blue, params.aprilTagLightTime);
                }
                if(robot.follower.isBusy())
                    pathTimer.reset();
                else if(Robot.params.greenPos != -1 || pathTimer.seconds() > params.aprilTagTimeout) {
                    if(robot.indexer.getNumBalls() == 3)
                        robot.transfer.setShootAll(true);
                    else if(robot.indexer.getNumBalls() == 0) {
                        robot.transfer.setShootAll(false);
                        updatePath(preCollect1Path);
                        robot.intake.setIntake(true);
                    }
                }
                break;
            case 2:
            case 6:
            case 9:
                if(!robot.follower.isBusy()) {
                    updatePath();
                    robot.follower.startTeleopDrive();
                }
                break;
            case 3:
                if(robot.indexer.getNumBalls() < 3 && robot.follower.getPose().getX() > collect1Pose.getX()) {
                    pidDriveY(collect1Pose);
                }
                else {
                    updatePath(shoot2Path, 2);
                    robot.intake.setIntake(false);
                    robot.indexer.rotate(robot.indexer.getAlignIndexerOffset());
                }
                break;
            case 4:
//                if(robot.follower.isBusy() && robot.follower.getPose().getX() > params.leverX)
//                    pathTimer.reset();
//                else if(pathTimer.seconds() > params.leverWait) {
//                    updatePath(shoot2PathLever);
//                    robot.indexer.rotate(robot.indexer.getAlignIndexerOffset());
//                }
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
                if(!robot.follower.isBusy()) {
                    if (robot.indexer.getNumBalls() > 0)
                        robot.transfer.setShootAll(true);
                    else {
                        robot.transfer.setShootAll(false);
                        updatePath(preCollect3Path);
                        robot.intake.setIntake(true);
                    }
                }
                break;
            case 10:
                if(robot.indexer.getNumBalls() < 3 && robot.follower.getPose().getX() > collect3Pose.getX())
                    pidDriveY(collect3Pose);
                else {
                    updatePath(shoot4Path);
                    robot.intake.setIntake(false);
                    robot.indexer.rotate(robot.indexer.getAlignIndexerOffset());
                }
                break;
            case 11:
                if(!robot.follower.isBusy()) {
                    if (robot.indexer.getNumBalls() > 0)
                        robot.transfer.setShootAll(true);
                    else
                        updatePath(endPath);
                }
                break;
            case 12:
                if(!robot.follower.isBusy())
                    return true;;
        }
        robot.follower.update();
        return false;
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
    private void updatePath(PathChain pathChain, int n) {
        robot.follower.followPath(pathChain);
        pathNum += n;
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
//        telemetry.addData("strafe", strafePower);
//        telemetry.addData("turn", turnPower);
//        telemetry.addData("goal heading", goalHeading);
//        telemetry.addData("cur heading", heading);
        robot.follower.setTeleOpDrive(params.collectDrivePower, strafePower, turnPower, true);
    }

}
