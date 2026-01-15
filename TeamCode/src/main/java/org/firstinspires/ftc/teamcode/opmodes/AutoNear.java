//package org.firstinspires.ftc.teamcode.opmodes;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.pedropathing.control.PIDFCoefficients;
//import com.pedropathing.control.PIDFController;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.robot.RGBLight;
//import org.firstinspires.ftc.teamcode.robot.Robot;
//import org.firstinspires.ftc.teamcode.robot.Transfer;
//import org.firstinspires.ftc.teamcode.utils.ColorSensorBall;
//import org.firstinspires.ftc.teamcode.utils.NullGamepadTracker;
//
//@Config
//public class AutoNear extends LinearOpMode {
//    public static class Params {
//        public double startX = 56.375, startY = 137, startA = -90;
//        public double endX = 38.825, endY = 31.8, endA = -90;
//        public double initShootX = 61, initShootY = 78, initShootA = -47;
//        public double shootX = 58, shootY = 78, shootA = -48.5;
//        public double shootXRed = 144-shootX + 3, shootYRed = 86;
//        public double shootFarX = 61, shootFarY = 16, shootFarA = -64.5;
//        public double shootFarARedOffset = -5;
//        public double collectX = 27, collectA = -175;
//        public double collectXRed = 144 - 24;
//        public double redCollectXOffset = 9;
//
//        public double preCollectX1 = 49, collect1Y = 94;
//
//        public double preCollectX2 = 46.5, collect2Y = 71;
//        public double preCollect2TX = 54, preCollect2TY = 65;
//        public double preCollectX3 = 46, collectY3 = 45;
//        public double preCollect3TX = 55, preCollect3TY = 70, preCollect3TA = -97;
//
//        public double preCollectX1Red = 108, preCollectX2Red = 100, preCollectX3Red = 107;
//        public double preCollectX2RedT = 86, preCollectX3RedT = 97;
//        public double preCollectY2RedT = 55, preCollectY3RedT = 70;
//        public double collectY1Red = 77, collectY2Red = 53, collectY3Red = 28;
//        public double abortTime = 1;
//
//        public double leverX = 25, leverY = 80, leverA = -180;
//
//        public double shoot2TX = 54, shoot2TY = 84;
//
//        public double collectDrivePower = 0.25, collectHeadingCorrectAmp = 0.3, collectStrafeCorrectAmp = -0.05;
//        public double aprilTagTimeout = 0.5, aprilTagLightTime = 0.8;
//        public double headingError = 7;
//        public double translationalKP = 0.06, headingKP = 0.02;
//        public double leverWait = 0.2;
//        public double headingConstraint = 5;
//        public double maxPower = 0.8, maxPower2 = 0.9, maxVel = 1, preShootWaitTime = 0.05;
//        public double maxCameraY = 125, cameraWait = 0.7;
//        public double shoot1Time = 3.5, preCollect1Time = 2.5, shoot2Time = 3.6, preCollect2Time = 3, shoot3Time = 3.3, preCollect3Time = 6;
//    }
//    public static Params params = new Params();
//    private Robot robot;
//    private Pose startPose, initShootPose, shootPose, preCollect1Pose, collect1Pose, leverPose, shoot2TPose, preCollect2Pose, preCollect2TPose, collect2Pose, preCollect3Pose, preCollect3TPose, collect3Pose, shootFarPose, endPose;
//    private Path shootAndCameraPath;
//    private PathChain preCollect1Path, leverPath, shoot2Path, preCollect2Path, shoot3Path, preCollect3Path, shoot4Path, endPath;
//    private PathChain collect1Path, collect2Path, collect3Path;
//    private int pathNum;
//    private ElapsedTime pathTimer;
//    private boolean rotatedYet;
//    private ElapsedTime autoTimer;
//    private PIDFController headingPid, translationalPid;
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Robot.params.greenPos = -1;
//        telemetry.setMsTransmissionInterval(11);
//
//        autoTimer = new ElapsedTime();
//        headingPid = new PIDFController(new PIDFCoefficients(params.headingKP, 0, 0, 0));
//        headingPid.setTargetPosition(0);
//        translationalPid = new PIDFController(new PIDFCoefficients(params.translationalKP, 0, 0, 0));
//        translationalPid.setTargetPosition(0);
//
//        startPose = new Pose(params.startX, params.startY, Math.toRadians(params.startA));
//        initShootPose = new Pose(params.initShootX, params.initShootY, Math.toRadians(params.initShootA));
//        shootPose = new Pose(params.shootX, params.shootY, Math.toRadians(params.shootA));
//        shootFarPose = new Pose(params.shootFarX, params.shootFarY, Math.toRadians(params.shootFarA));
//        preCollect1Pose = new Pose(params.preCollectX1, params.collect1Y, Math.toRadians(params.collectA));
//        collect1Pose = new Pose(params.collectX, params.collect1Y, Math.toRadians(params.collectA));
//        leverPose = new Pose(params.leverX, params.leverY, Math.toRadians(params.leverA));
//        shoot2TPose = new Pose(params.shoot2TX, params.shoot2TY);
//        preCollect2Pose = new Pose(params.preCollectX2, params.collect2Y, Math.toRadians(params.collectA));
//        preCollect2TPose = new Pose(params.preCollect2TX, params.preCollect2TY);
//        collect2Pose = new Pose(params.collectX, params.collect2Y, Math.toRadians(params.collectA));
//        preCollect3Pose = new Pose(params.preCollectX3, params.collectY3, Math.toRadians(params.collectA));
//        preCollect3TPose = new Pose(params.preCollect3TX, params.preCollect3TY, Math.toRadians(params.preCollect3TA));
//        collect3Pose = new Pose(params.collectX, params.collectY3, Math.toRadians(params.collectA));
//        endPose = new Pose(params.endX, params.endY, Math.toRadians(params.endA));
//
//        if(Robot.params.red) {
//            params.collectStrafeCorrectAmp *= -1;
//            params.collectHeadingCorrectAmp *= -1;
//            startPose = mirror(startPose);
//            initShootPose = new Pose(params.shootXRed, params.shootYRed, mirrorA(initShootPose.getHeading()));
//            shootPose = new Pose(params.shootXRed, params.shootYRed, mirrorA(shootPose.getHeading()));
//            preCollect1Pose = new Pose(params.preCollectX1Red, params.collectY1Red, mirrorA(preCollect1Pose.getHeading()));
//            collect1Pose = mirror(collect1Pose);
//            collect1Pose = new Pose(collect1Pose.getX() + params.redCollectXOffset, params.collectY1Red);
//            leverPose = mirror(leverPose);
//            shoot2TPose = mirror(shoot2TPose);
//            preCollect2Pose = new Pose(params.preCollectX2Red, params.collectY2Red, mirrorA(preCollect2Pose.getHeading()));
//            preCollect2TPose = new Pose(params.preCollectX2RedT, params.preCollectY2RedT, mirrorA(preCollect2TPose.getHeading()));
//            collect2Pose = mirror(collect2Pose);
//            collect2Pose = new Pose(collect2Pose.getX() + params.redCollectXOffset, params.collectY2Red);
//            preCollect3Pose = new Pose(params.preCollectX3Red, params.collectY3Red, Math.toRadians(-5));
//            preCollect3TPose = new Pose(params.preCollectX3RedT, params.preCollectY3RedT, Math.toRadians(-90));
//            collect3Pose = new Pose(180-collect3Pose.getX() + params.redCollectXOffset, params.collectY3Red, Math.toRadians(-5));
//            shootFarPose = mirror(shootFarPose);
//            shootFarPose = new Pose(shootFarPose.getX(), shootFarPose.getY(), shootFarPose.getHeading() + params.shootFarARedOffset);
//            endPose = mirror(endPose);
//        }
//        telemetry.addData("start", printPose(startPose));
//        telemetry.addData("initShoot", printPose(initShootPose));
//        telemetry.addData("shoot", printPose(shootPose));
//        telemetry.addData("preC1", printPose(preCollect1Pose));
//        telemetry.addData("c1", printPose(collect1Pose));
//        telemetry.addData("shoot2T", printPose(shoot2TPose));
//        telemetry.addData("preC2", printPose(preCollect2Pose));
//        telemetry.addData("preC2T", printPose(preCollect2TPose));
//        telemetry.addData("c2", printPose(collect2Pose));
//        telemetry.addData("preC3", printPose(preCollect3Pose));
//        telemetry.addData("preC3T", printPose(preCollect3TPose));
//        telemetry.addData("c3", printPose(collect3Pose));
//        telemetry.addData("shootFar", printPose(shootFarPose));
//        telemetry.addData("end", printPose(endPose));
//
//        robot = new Robot(hardwareMap, telemetry, new NullGamepadTracker(), new NullGamepadTracker(), startPose);
//        robot.indexer.resetEncoder();
//        telemetry.addData("ball list", robot.indexer.getLabeledBalls());
//        telemetry.addData("intake i", robot.indexer.getIntakeI());
//        telemetry.addData("preCollect 1 heading", preCollect1Pose.getHeading());
//        telemetry.addData("team", Robot.params.red ? "RED" : "BLUE");
//        robot.indexer.setAutoBallList(1, ColorSensorBall.BallColor.P, ColorSensorBall.BallColor.P, ColorSensorBall.BallColor.G);
//        robot.indexer.setAutoRotate(true);
//        robot.shooter.setZone(0);
//        robot.shooter.setResting(false);
//        robot.intake.setIntakeSafely(false);
//        robot.transfer.setShootSafely(true);
//
//        shootAndCameraPath = new Path(new BezierLine(startPose, initShootPose));
//        shootAndCameraPath.setLinearHeadingInterpolation(startPose.getHeading(), initShootPose.getHeading());
//        shootAndCameraPath.setTimeoutConstraint(params.shoot1Time);
//
//        robot.follower.setMaxPower(params.maxPower);
//        preCollect1Path = robot.follower.pathBuilder()
//                .addPath(new BezierLine(shootPose, preCollect1Pose))
//                .setConstantHeadingInterpolation(preCollect1Pose.getHeading())
//                .setHeadingConstraint(Math.toRadians(params.headingError))
//                .setTimeoutConstraint(params.preCollect1Time)
//                .build();
//        collect1Path = robot.follower.pathBuilder()
//                .addPath(new BezierLine(preCollect1Pose, collect1Pose))
//                .setConstantHeadingInterpolation(collect1Pose.getHeading())
//                .build();
//
//        robot.follower.setMaxPower(1);
//        leverPath = robot.follower.pathBuilder()
//                .addPath(new BezierLine(collect1Pose, leverPose))
//                .setLinearHeadingInterpolation(collect1Pose.getHeading(), leverPose.getHeading())
//                .build();
//        shoot2Path = robot.follower.pathBuilder()
//                .addPath(new BezierLine(collect1Pose, shootPose))
//                .setLinearHeadingInterpolation(collect1Pose.getHeading(), shootPose.getHeading())
//                .setHeadingConstraint(Math.toRadians(params.headingConstraint))
//                .setTimeoutConstraint(params.shoot2Time)
//                .build();
//        preCollect2Path = robot.follower.pathBuilder()
//                .addPath(new BezierCurve(shootPose, preCollect2TPose, preCollect2Pose))
//                .setLinearHeadingInterpolation(shootPose.getHeading(), preCollect2Pose.getHeading())
//                .setHeadingConstraint(Math.toRadians(params.headingError))
//                .setTimeoutConstraint(params.preCollect2Time)
//                .build();
//        robot.follower.setMaxPower(params.collectDrivePower);
//        collect2Path = robot.follower.pathBuilder()
//                .addPath(new BezierLine(preCollect2Pose, collect2Pose))
//                .setConstantHeadingInterpolation(collect2Pose.getHeading())
//                .build();
//        robot.follower.setMaxPower(1);
//        shoot3Path = robot.follower.pathBuilder()
//                .addPath(new BezierLine(collect2Pose, shootPose))
//                .setLinearHeadingInterpolation(collect2Pose.getHeading(), shootPose.getHeading())
//                .setHeadingConstraint(Math.toRadians(params.headingConstraint))
//                .setTimeoutConstraint(params.shoot3Time)
//                .build();
//        preCollect3Path = robot.follower.pathBuilder()
//                .addPath(new BezierLine(shootPose, preCollect3TPose))
//                .setConstantHeadingInterpolation(preCollect3TPose.getHeading())
//                .addPath(new BezierLine(preCollect3TPose, preCollect3Pose))
//                .setLinearHeadingInterpolation(preCollect3TPose.getHeading(), preCollect3Pose.getHeading())
//                .setTimeoutConstraint(params.preCollect3Time)
//                .build();
//        robot.follower.setMaxPower(params.collectDrivePower);
//        collect3Path = robot.follower.pathBuilder()
//                .addPath(new BezierLine(preCollect3Pose, collect3Pose))
//                .setConstantHeadingInterpolation(collect3Pose.getHeading())
//                .build();
//        robot.follower.setMaxPower(1);
//        shoot4Path = robot.follower.pathBuilder()
//                .addPath(new BezierLine(collect3Pose, shootFarPose))
//                .setConstantHeadingInterpolation(shootFarPose.getHeading())
//                .setHeadingConstraint(Math.toRadians(params.headingConstraint))
//                .setVelocityConstraint(params.maxVel)
//                .build();
//        endPath = robot.follower.pathBuilder()
//                .addPath(new BezierLine(shootPose, endPose))
//                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
//                .build();
//
//        headingPid = new PIDFController(robot.follower.constants.coefficientsHeadingPIDF);
//        translationalPid = new PIDFController(robot.follower.constants.coefficientsTranslationalPIDF);
//
//        pathNum = 0;
//        pathTimer = new ElapsedTime();
//        telemetry.addData("ready", "");
//        telemetry.addData("ball list", robot.indexer.getLabeledBalls());
//        telemetry.addData("intake i", robot.indexer.getIntakeI());
//        telemetry.update();
//        robot.rgbLight.updateInit();
//        waitForStart();
//        autoTimer.reset();
//        while(opModeIsActive()) {
//            robot.updateSubsystems();
//            if (updatePedroAuto())
//                break;
//            telemetry.addData("green i", Robot.params.greenPos);
//            telemetry.addData("pose", Math.floor(robot.getX()*10)/10 + ", " + Math.floor(robot.getY()*10)/10 + ", " + Math.floor(robot.getHeading() * 180 / Math.PI*10)/10);
//            telemetry.addData("path i & timer", pathNum + ", " + pathTimer.seconds());
//            telemetry.addData("follower busy", robot.follower.isBusy());
//            telemetry.addData("shooter vel", Math.floor(robot.shooter.getShooterVelocity() * 100)/100);
//            telemetry.addData("hood pos", robot.shooter.getHoodPos());
//            telemetry.addData("target shooter vel", robot.shooter.getTargetMotorVel());
//            telemetry.addData("indexer ready to shoot", robot.indexer.prettyMuchStaticShoot());
//            telemetry.addData("num balls", robot.indexer.getNumBalls());
//            telemetry.addData("intake i", robot.indexer.getIntakeI());
//            telemetry.addData("intake state", robot.intake.getIntakeState());
//            telemetry.addData("intake power", robot.intake.getIntakePower());
//            telemetry.addData("transfer state", robot.transfer.getTransferState());
//            telemetry.update();
//        }
//    }
//    private boolean updatePedroAuto() {
//        switch(pathNum) {
//            case 0:
//                updatePath(shootAndCameraPath);
//                break;
//            case 1:
//                robot.updatePattern();
//                if(robot.follower.isBusy())
//                    pathTimer.reset();
//                else {
//                    if(!rotatedYet) {
//                        rotatedYet = true;
//                        robot.indexer.rotate(robot.indexer.getAlignShootOffset());
//                        robot.rgbLight.setState(RGBLight.LightState.SET, RGBLight.params.yellow, params.aprilTagLightTime);
//                    }
//                    else if(robot.indexer.getNumBalls() > 0)
//                        robot.transfer.setShootAll(true);
//                    else if(robot.transfer.getTransferState() != Transfer.TransferState.TRANSFERRING) {
//                        robot.transfer.setShootAll(false);
//                        robot.follower.setMaxPower(params.maxPower);
//                        updatePath(preCollect1Path);
//                        robot.intake.setIntake(true);
//                    }
//                }
//                break;
//            case 2:
//                if((robot.getX() < preCollect1Pose.getX() && !Robot.params.red) || !robot.follower.isBusy()) {
//                    robot.follower.setMaxPower(params.collectDrivePower);
//                    updatePath(collect1Path);
//                }
//                break;
//            case 3:
//                if(!robot.follower.isBusy()) {
//                    robot.follower.setMaxPower(1);
//                    updatePath(shoot2Path, 2);
//                    robot.intake.setIntake(false);
//                    robot.indexer.rotate(robot.indexer.getAlignShootOffset());
//                    robot.shooter.setResting(false);
//                }
//                break;
//            case 4:
////                if(robot.follower.isBusy() && robot.follower.getPose().getX() > params.leverX)
////                    pathTimer.reset();
////                else if(pathTimer.seconds() > params.leverWait) {
////                    updatePath(shoot2PathLever);
////                    robot.indexer.rotate(robot.indexer.getAlignIndexerOffset());
////                }
//                break;
//            case 5:
//                if(robot.follower.isBusy())
//                    pathTimer.reset();
//                else if(pathTimer.seconds() > params.preShootWaitTime) {
//                    if (robot.indexer.getNumBalls() > 0)
//                        robot.transfer.setShootAll(true);
//                    else if(robot.transfer.getTransferState() != Transfer.TransferState.TRANSFERRING) {
//                        robot.transfer.setShootAll(false);
//                        updatePath(preCollect2Path);
//                        robot.intake.setIntake(true);
//                    }
//                }
//                break;
//            case 6:
//                if((robot.getX() < preCollect2Pose.getX() && !Robot.params.red) || !robot.follower.isBusy()) {
//                    robot.follower.setMaxPower(params.collectDrivePower);
//                    updatePath(collect2Path);
//                }
//            case 7:
//                if(!robot.follower.isBusy()) {
//                    robot.follower.setMaxPower(1);
//                    updatePath(shoot3Path);
//                    robot.intake.setIntake(false);
//                    robot.indexer.rotate(robot.indexer.getAlignShootOffset());
//                    robot.shooter.setResting(false);
//                }
//
//                break;
//            case 8:
//                if(robot.follower.isBusy())
//                    pathTimer.reset();
//                else if(pathTimer.seconds() > params.preShootWaitTime) {
//                    if (robot.indexer.getNumBalls() > 0)
//                        robot.transfer.setShootAll(true);
//                    else if(robot.transfer.getTransferState() != Transfer.TransferState.TRANSFERRING) {
//                        robot.transfer.setShootAll(false);
//                        robot.follower.setMaxPower(params.maxPower2);
//                        updatePath(preCollect3Path);
//                        robot.intake.setIntake(true);
//                    }
//                }
//                break;
//            case 9:
//                if((robot.getX() < preCollect3Pose.getX() && !Robot.params.red) || !robot.follower.isBusy()) {
//                    robot.follower.setMaxPower(params.collectDrivePower);
//                    updatePath(collect3Path);
//                }
//                break;
//            case 10:
//                if(autoTimer.seconds() > 30 - params.abortTime) {
//                    updatePath(3);
//                    robot.follower.startTeleopDrive();
//                }
//                else if(!robot.follower.isBusy()) {
//                    robot.follower.setMaxPower(1);
//                    updatePath(shoot4Path);
//                    robot.intake.setIntake(false);
//                    robot.indexer.rotate(robot.indexer.getAlignShootOffset());
//                    robot.shooter.setZone(2);
//                    robot.shooter.setResting(false);
//                }
//                break;
//            case 11:
//                if(robot.follower.isBusy())
//                    pathTimer.reset();
//                else {
//                    if(autoTimer.seconds() > 30 - params.abortTime)
//                        updatePath();
//                    else if (robot.indexer.getNumBalls() > 0) {
//                        if (pathTimer.seconds() > params.preShootWaitTime)
//                            robot.transfer.setShootAll(true);
//                    }
//                    else if(robot.transfer.getTransferState() != Transfer.TransferState.TRANSFERRING) {
//                        updatePath();
//                    }
//                }
//                break;
//            case 12:
//                updatePath(endPath);
//                robot.transfer.setShootAll(false);
//                robot.shooter.setResting(true);
//                MainTele.x = robot.getX();
//                MainTele.y = robot.getY();
//                MainTele.a = robot.getHeading();
//                break;
//            case 13:
//                MainTele.x = robot.getX();
//                MainTele.y = robot.getY();
//                MainTele.a = robot.getHeading();
//                robot.indexer.rotate(robot.indexer.getAlignCollectOffset());
//                break;
//        }
//        robot.follower.update();
//        return false;
//    }
//
//    private void updatePath(Path path) {
//        robot.follower.followPath(path);
//        pathNum++;
//        pathTimer.reset();
//    }
//    private void updatePath(PathChain pathChain) {
//        robot.follower.followPath(pathChain);
//        pathNum++;
//        pathTimer.reset();
//    }
//    private void updatePath(int n) {
//        pathNum += n;
//        pathTimer.reset();
//    }
//    private void updatePath(PathChain pathChain, int n) {
//        robot.follower.followPath(pathChain);
//        pathNum += n;
//        pathTimer.reset();
//    }
//    private void updatePath() {
//        pathNum++;
//        pathTimer.reset();
//    }
//    private Pose mirror(Pose pose) {
//        return new Pose(144 - pose.getX(), pose.getY(), Math.PI - pose.getHeading());
//    }
//    private void pidDriveY(Pose pose) {
//        double strafePower = (pose.getY() - robot.getY()) * params.collectStrafeCorrectAmp;
//        double goalHeading, heading;
//
//        goalHeading = (pose.getHeading() + Math.PI * 2) % (Math.PI * 2);
//        heading = (robot.getHeading() + Math.PI * 2) % (Math.PI * 2);
//
//        double turnPower = (goalHeading - heading) * params.collectHeadingCorrectAmp;
////        telemetry.addData("strafe", strafePower);
////        telemetry.addData("turn", turnPower);
////        telemetry.addData("goal heading", goalHeading);
////        telemetry.addData("cur heading", heading);
//        robot.follower.setTeleOpDrive(params.collectDrivePower, strafePower, turnPower, true);
//    }
//    private String printPose(Pose pose) {
//        return Math.floor(pose.getX() * 10) / 10 + ", " + Math.floor(pose.getY() * 10) / 10 + ", " + Math.floor(pose.getHeading() * 10) / 10;
//    }
//    private double mirrorA(double a) {
//        double angle = Math.PI - a;
//        if(angle > Math.PI)
//            angle -= Math.PI * 2;
//        return angle;
//    }
//}
