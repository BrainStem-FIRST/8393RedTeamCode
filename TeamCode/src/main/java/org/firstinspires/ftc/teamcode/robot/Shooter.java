package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.Regression;

import java.util.function.Supplier;

@Config
public class Shooter {
    public static class Params {
        public double kP = 0.05, kI = 0, kD = 0.003, maxShooterPower = 0.8;
        public double farDist = 145;
        public double nearDist = 75;
        public double midDist = (farDist + nearDist) / 2;

        public double initHoodPosFar = 0.08;
        public double initHoodPosNear = 0.43;

        public double shooterVelocityFar = 395, initShooterPowerFar = 0.65;
        public double shooterVelocityNear = 290, initShooterPowerNear = 0.42;
        public double t = 0.75;
        public int minHoodPwm = 1440, maxHoodPwm = 1010;
        public double hoodFarRegressM = 0, hoodFarRegressB = 0;
        public double hoodNearRegressM = 0.1, hoodNearRegressB = 0.1;
        public double manualHoodInc = 0.01, autoHoodInc = 0.015, manualShooterInc = 5;
    }
    public static Params params = new Params();
    private final Robot robot;
    private boolean shouldShoot;
    private final DcMotorEx motor1;
    private final DcMotorEx motor2;

    private final ServoImplEx hoodServo;
    private PIDController shooterPid;

    // NL and NR signify the left and right near positions
    private double targetMotorPower;
    private double targetMotorVel;
    private double targetHoodPos;
    private double motorVelOffset, hoodOffset;
    private Regression hoodFarRegress, hoodNearRegress; // regressions output desired hood position as function of motor velocity (in degrees/s)
    private double totalTime;
    private int zone;
    public Shooter(Robot robot) {
        this.robot = robot;

        motor1 = robot.hardwareMap.get(DcMotorEx.class, "shooter1");
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor2 = robot.hardwareMap.get(DcMotorEx.class, "shooter2");
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hoodServo = robot.hardwareMap.get(ServoImplEx.class, "hoodServo");
        hoodServo.setPwmRange(new PwmControl.PwmRange(params.minHoodPwm, params.maxHoodPwm));

        hoodFarRegress = new Regression(params.hoodFarRegressM, params.hoodFarRegressB);
        hoodNearRegress = new Regression(params.hoodNearRegressM, params.hoodNearRegressB);

        targetMotorPower = 0;
        targetHoodPos = params.initHoodPosFar;
        targetMotorVel = params.shooterVelocityFar;
        totalTime = 0;
        zone = 1;

        shooterPid = new PIDController(params.kP, params.kI, params.kD);
        shooterPid.setOutputBounds(-params.maxShooterPower, params.maxShooterPower);
        shooterPid.setTarget(targetMotorVel);
    }

    public void update(double totalTime) {
        this.totalTime = totalTime;
        if(robot.g1.isFirstY())
            shouldShoot = !shouldShoot;
        if(robot.g1.isFirstBack())
            zone *= -1;

        double dist = distance();
        // automatically setting shooter hood and power
        if (zone == 1) {
            targetMotorVel = params.shooterVelocityFar + motorVelOffset;
            shooterPid.setTarget(targetMotorVel);
            targetHoodPos = Range.clip(params.initHoodPosFar + hoodFarRegress.f(getShooterVelocity()-params.shooterVelocityFar) + hoodOffset, 0, 1);
        }
        else if(zone == -1) {
            targetMotorVel = params.shooterVelocityNear + motorVelOffset;
            shooterPid.setTarget(targetMotorVel);
//            targetHoodPos = hoodNearRegress.f(getShooterVelocity());
            targetHoodPos = params.initHoodPosNear + hoodOffset;
        }

        // setting hood position
        if(robot.g1.isFirstDpadUp()) {
            hoodOffset += params.manualHoodInc;
        }
        else if(robot.g1.isFirstDpadDown()) {
            hoodOffset -= params.manualHoodInc;
        }
//        else if(robot.indexer.justTransferred())
//            if(robot.indexer.getNumBalls() == 0)
//                hoodOffset -= params.autoHoodInc * 3;
//            else
//                hoodOffset += params.autoHoodInc;

        // setting shooter power
        if(robot.g1.isFirstDpadRight()) {
            motorVelOffset += params.manualShooterInc;
            shooterPid.setTarget(targetMotorVel + motorVelOffset);
        }
        else if(robot.g1.isFirstDpadLeft()) {
            motorVelOffset -= params.manualShooterInc;
            shooterPid.setTarget(targetMotorVel + motorVelOffset);
        }

        if(shouldShoot) {
            double idealPower = dist > params.midDist ? params.initShooterPowerFar : params.initShooterPowerNear;
            setShooterPower(shooterPid.update(getShooterVelocity()) * params.t + idealPower * (1 - params.t));
        }
        else
            setShooterPower(0);
        hoodServo.setPosition(targetHoodPos);
    }
    public double getHoodPos() {
        return hoodServo.getPosition();
    }
    public int getZone() {
        return zone;
    }
    private void setShooterVel(double vel) {
        motor1.setVelocity(vel, AngleUnit.DEGREES);
        motor2.setVelocity(-vel, AngleUnit.DEGREES);
    }
    private void setShooterPower(double power) {
        motor1.setPower(power);
        motor2.setPower(-power);
    }
    public double getMotorVelOffset() {
        return motorVelOffset;
    }
    public double getHoodOffset() {
        return hoodOffset;
    }

    public double distance(){
        return Math.sqrt(Math.pow(robot.follower.getPose().getX()-robot.goalX, 2) + Math.pow(robot.follower.getPose().getY()-robot.goalY, 2));
    }
    public double getShooterVelocity() {
        return (motor1.getVelocity(AngleUnit.DEGREES) - motor2.getVelocity(AngleUnit.DEGREES)) * 0.5;
    }
    public double getMotor2Velocity() {
        return motor2.getVelocity(AngleUnit.DEGREES);
    }
    public double getShooterPower() {
        return motor1.getPower();
    }
    public double getTargetMotorVel() {
        return targetMotorVel;
    }
    public boolean getShouldShoot() {
        return shouldShoot;
    }
    public double getShooterMiliAmps() {
        return motor1.getCurrent(CurrentUnit.MILLIAMPS);
    }
}