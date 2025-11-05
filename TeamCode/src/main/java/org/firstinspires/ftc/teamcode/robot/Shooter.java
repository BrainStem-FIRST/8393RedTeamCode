package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.Cubic;
import org.firstinspires.ftc.teamcode.utils.ExpoRegression;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.QuadRegression;

@Config
public class Shooter {
    public static class Params {
        public double kP = 0.05, kI = 0, kD = 0.003, maxShooterPower = 0.8, startupPower = 1;
        public double startupDist = 100; // if velocity is less than target by this amount, i only apply startup power
        public double farDistTele = 151.7, farDistAuto = 145; // far dist auto is guessed
        public double nearDist = 65;
        public double midDist = (farDistTele + nearDist) / 2;

        public double initHoodPosFar = 0.08;

        public double shooterVelocityFarTele = 410, shooterPowerFarTele = 0.68;
        public double shooterVelocityNear = 280, shooterPowerNear = 0.45;

        public double minShooterVelFarTele = 375, minShooterVelNear = 250;
        public double maxShooterVelFarTele = 430, maxShooterVelNear = 290;
        public double restPowerOffset = 0.2;
        public double t = 0.2;
        public int minHoodPwm = 1800, maxHoodPwm = 1400;

        public double hoodFarA = -0.000007323075, hoodFarB = 0.008641343, hoodFarC = -3.401257, hoodFarD = 446.75658; //y=-0.000007323075x^{3}+0.008641343x^{2}-3.401257x+446.75658
        public double hoodNearRegressA = 0.0009, hoodNearRegressB = -0.4767, hoodNearRegressC = 63.8395;//y=0.0009x^{2}-0.4767x+63.8395

        public double manualHoodInc = 0.01, manualShooterInc = 0.01;
        public double hoodOffset;
    }
    public static Params params = new Params();
    private final Robot robot;
    private boolean shouldShoot;
    private final DcMotorEx motor1;
    private final DcMotorEx motor2;

    private final ServoImplEx hoodServo;
    private final PIDController shooterPid;

    // NL and NR signify the left and right near positions
    private double curMotorPower, targetMotorPower;
    private double curMotorVel, targetMotorVel, minMotorVel, maxMotorVel;
    private boolean powerUpdated, velUpdated; // caching functionality
    private double targetHoodPos;
    private double motorPowerOffset, hoodOffset;
    private final Cubic hoodTeleFarRegress;
    private final QuadRegression hoodNearRegress; // regressions output desired hood position as function of motor velocity (in degrees/s)
    private int zone;
    private double shooterPower, prevShooterPower, prevHoodPos;
    private boolean resting;
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

        hoodTeleFarRegress = new Cubic(params.hoodFarA, params.hoodFarB, params.hoodFarC, params.hoodFarD);
        hoodNearRegress = new QuadRegression(params.hoodNearRegressA, params.hoodNearRegressB, params.hoodNearRegressC);

        targetMotorPower = 0;
        targetHoodPos = params.initHoodPosFar;
        targetMotorVel = params.shooterVelocityFarTele;
        zone = 0;

        shooterPid = new PIDController(params.kP, params.kI, params.kD);
        shooterPid.setOutputBounds(-params.maxShooterPower, params.maxShooterPower);
        shooterPid.setTarget(targetMotorVel);

        hoodOffset = params.hoodOffset;
        shouldShoot = true;
    }
    public void resetCaches() {
        powerUpdated = false;
        velUpdated = false;
    }
    public void update() {
        if(robot.g1.isFirstY())
            shouldShoot = !shouldShoot;
        // zone changers
        if(robot.g1.isFirstDpadUp())
            zone = 0;
        else if(robot.g1.isFirstDpadDown())
            zone = 1;

        // automatically setting shooter hood and power
        if(zone == 0) {
            targetMotorVel = params.shooterVelocityNear;
            minMotorVel = params.minShooterVelNear;
            maxMotorVel = params.maxShooterVelNear;
            targetMotorPower = params.shooterPowerNear;
            shooterPid.setTarget(targetMotorVel);
            targetHoodPos = Range.clip(Range.clip(hoodNearRegress.f(getShooterVelocity()), 0, 1) + hoodOffset, 0, 1);
        }
        else {
            targetMotorVel = params.shooterVelocityFarTele;
            minMotorVel = params.minShooterVelFarTele;
            maxMotorVel = params.maxShooterVelFarTele;

            targetMotorPower = params.shooterPowerFarTele;
            shooterPid.setTarget(targetMotorVel);
            targetHoodPos = Range.clip(Range.clip(hoodTeleFarRegress.f(getShooterVelocity()), 0, 1) + hoodOffset, 0, 1);
        }

        // setting hood position
        if(robot.g2.isFirstDpadUp())
            hoodOffset += params.manualHoodInc;
        else if(robot.g2.isFirstDpadDown())
            hoodOffset -= params.manualHoodInc;

        // setting shooter power
        if(robot.g2.isFirstDpadRight())
            motorPowerOffset += params.manualShooterInc;

        else if(robot.g2.isFirstDpadLeft())
            motorPowerOffset -= params.manualShooterInc;

        // do not adjust after shoot button has been pressed
        shooterPower = 0;
        if (shouldShoot) {
            if(resting)
                shooterPower = targetMotorPower - params.restPowerOffset;
            else if(targetMotorVel - getShooterVelocity() > params.startupDist)
                shooterPower = params.startupPower;
            else {
                // params.t is a value between 0 and 1 (I have it set to 0.9)
                double pid = shooterPid.update(getShooterVelocity());
                robot.telemetry.addData("shooter pid", pid);
                shooterPower = pid * params.t + (targetMotorPower + motorPowerOffset) * (1 - params.t);
            }
        }

        if(shooterPower != prevShooterPower)
            setShooterPower(shooterPower);
        if(targetHoodPos != prevHoodPos)
            hoodServo.setPosition(targetHoodPos);

        prevShooterPower = shooterPower;
        prevHoodPos = targetHoodPos;
    }
    public double getHoodPos() {
        return hoodServo.getPosition();
    }
    public int getZone() {
        return zone;
    }
    public void setZone(int zone) {
        this.zone = zone;
    }
    private void setShooterPower(double power) {
        motor1.setPower(power);
        motor2.setPower(-power);
    }
    public double distance(){
        return Math.sqrt(Math.pow(robot.follower.getPose().getX()-Robot.params.goalX, 2) + Math.pow(robot.follower.getPose().getY()-Robot.params.goalY, 2));
    }
    public double getShooterVelocity() {
        if(!velUpdated) {
            velUpdated = true;
            curMotorVel = (motor1.getVelocity(AngleUnit.DEGREES) - motor2.getVelocity(AngleUnit.DEGREES)) * 0.5;
        }
        return curMotorVel;
    }
    public double getMinShooterVel() {
        return minMotorVel;
    }
    public double getMaxShooterVel() {
        return maxMotorVel;
    }
    public double getShooterPower() {
        if(!powerUpdated) {
            powerUpdated = true;
            curMotorPower = motor1.getPower();
        }
        return curMotorPower;
    }
    public double getTargetMotorVel() {
        return targetMotorVel;
    }
    public double getTargetMotorPower() {
        return targetMotorPower;
    }
    public boolean getShouldShoot() {
        return shouldShoot;
    }
    public void setShouldShoot(boolean shouldShoot) {
        this.shouldShoot = shouldShoot;
    }
    public void setResting(boolean resting) {
        this.resting = resting;
    }
    public boolean isResting() {
        return resting;
    }
}