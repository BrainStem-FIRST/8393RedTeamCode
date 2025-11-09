package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.math.Line;

@Config
public class Shooter {
    public static class Params {
        public double kP = 0.022, kI = 0, kD = 0.004, maxShooterPower = 1, startupPower = 1;
        public double startupDist = 50; // if velocity is less than target by this amount, i only apply startup power
        public double farDistTele = 151.7, farDistAuto = 145; // far dist auto is guessed
        public double nearDist = 65;
        public double midDist = (farDistTele + nearDist) / 2;

        public double shooterVelFarAuto = 380;
        public double shooterVelocityFar = 380, shooterPowerFar = 0.6;
        public double shooterVelocityNear = 275, shooterPowerNear = 0.47;

        public double minShooterVelFarAuto = 370, maxShooterVelFarAuto = 400;
        public double minShooterVelFar = 370, maxShooterVelFar = 400;
        public double minShooterVelNear = 260, maxShooterVelNear = 300;
        public double restPowerOffset = 0.5;
        public int minHoodPwm = 1800, maxHoodPwm = 1400;
        public double hoodNearM = -0.0113448, hoodNearB = 3.64514; //y=-0.0113448x+3.64514
        public double hoodFarM = -0.00223382, hoodFarB = 0.865; //y=-0.00223382x+0.900779
        public double hoodFarAutoM = -0.00223382, hoodFarAutoB = 0.865;
        public double hoodNearOffset = 13;

        public double manualHoodInc = 0.05, manualShooterInc = 2.5;
        public double hoodOffset;
    }
    public static Params params = new Params();
    private final Robot robot;
    private final DcMotorEx motor1;
    private final DcMotorEx motor2;

    private final ServoImplEx hoodServo;
    private final PIDController shooterPid;

    // NL and NR signify the left and right near positions
    private double curMotorPower, targetMotorPower;
    private double curMotorVel, targetMotorVel, minMotorVel, maxMotorVel;
    private boolean powerUpdated, velUpdated; // caching functionality
    private double targetHoodPos;
    private double hoodOffset;
    private final Line hoodFarAutoRegress, hoodFarRegress, hoodNearRegress; // regressions output desired hood position as function of motor velocity (in degrees/s)
    private int zone;
    private double shooterPower, prevShooterPower, prevHoodPos;
    private boolean resting, hoodLocked;
    private double goalDist;
    private boolean zoneChangeCued;
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

        hoodFarAutoRegress = new Line(params.hoodFarAutoM, params.hoodFarAutoB);
        hoodFarRegress = new Line(params.hoodFarM, params.hoodFarB);
        hoodNearRegress = new Line(params.hoodNearM, params.hoodNearB);

        targetMotorPower = 0;
        targetHoodPos = 1;
        targetMotorVel = params.shooterVelocityNear;
        minMotorVel = params.minShooterVelNear;
        maxMotorVel = params.maxShooterVelNear;
        zone = 0;

        shooterPid = new PIDController(params.kP, params.kI, params.kD);
        shooterPid.setOutputBounds(-params.maxShooterPower, params.maxShooterPower);
        shooterPid.setTarget(targetMotorVel);

        hoodOffset = params.hoodOffset;
        resting = false;
    }
    public void resetCaches() {
        powerUpdated = false;
        velUpdated = false;
    }
    public void update() {
        goalDist = Math.sqrt(Math.pow(robot.getX() - robot.getGoalX(), 2) + Math.pow(robot.getY() - robot.getGoalY(), 2));

        if(robot.g2.isFirstStart())
            resting = !resting;
        // zone changers
        if(robot.g2.isFirstDpadUp() || (zoneChangeCued && zone == 0)) {
            zone = 0;
            targetMotorVel = params.shooterVelocityNear;
            minMotorVel = params.minShooterVelNear;
            maxMotorVel = params.maxShooterVelNear;
            targetMotorPower = params.shooterPowerNear;
            shooterPid.setTarget(targetMotorVel);
        }
        else if(robot.g2.isFirstDpadDown() || (zoneChangeCued && zone == 1)) {
            zone = 1;
            targetMotorVel = params.shooterVelocityFar;
            minMotorVel = params.minShooterVelFar;
            maxMotorVel = params.maxShooterVelFar;
            targetMotorPower = params.shooterPowerFar;
            shooterPid.setTarget(targetMotorVel);
            zoneChangeCued = false;
        }
        else if(zoneChangeCued && zone == 2) {
            targetMotorVel = params.shooterVelFarAuto;
            minMotorVel = params.minShooterVelFarAuto;
            maxMotorVel = params.maxShooterVelFarAuto;
            targetMotorPower = params.shooterPowerFar;
            shooterPid.setTarget(targetMotorVel);
            zoneChangeCued = false;
        }
        // setting shooter power
//        if(robot.g2.isFirstDpadRight()) {
//            targetMotorVel += params.manualShooterInc;
//            shooterPid.setTarget(targetMotorVel);
//        }
//        else if(robot.g2.isFirstDpadLeft()) {
//            targetMotorVel -= params.manualShooterInc;
//            shooterPid.setTarget(targetMotorVel);
//        }

        // automatically setting hood
        if(!hoodLocked) {
            if (zone == 0)
                targetHoodPos = Range.clip(Range.clip(hoodNearRegress.eval(getShooterVelocity() + params.hoodNearOffset), 0, 1) + hoodOffset, 0, 1);
            else if(zone == 1)
                targetHoodPos = Range.clip(Range.clip(hoodFarRegress.eval(getShooterVelocity()), 0, 1) + hoodOffset, 0, 1);
            else
                targetHoodPos = Range.clip(Range.clip(hoodFarAutoRegress.eval(getShooterVelocity()), 0, 1) + hoodOffset, 0, 1);
        }
        // manually setting hood
//        if(robot.g2.isFirstDpadUp())
//            hoodOffset += params.manualHoodInc;
//        else if(robot.g2.isFirstDpadDown())
//            hoodOffset -= params.manualHoodInc;

        // calculating shooter power
        shooterPower = 0;
        if(resting)
            shooterPower = Math.max(targetMotorPower - params.restPowerOffset, 0);
        else {
            if (targetMotorVel - getShooterVelocity() > params.startupDist)
                shooterPower = params.startupPower;
            else {
                // params.t is a value between 0 and 1 (I have it set to 0.9)
                double pid = shooterPid.update(getShooterVelocity());
                shooterPower = pid + targetMotorPower;
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
        zoneChangeCued = true;
        this.zone = zone;
    }
    private void setShooterPower(double power) {
        motor1.setPower(power);
        motor2.setPower(-power);
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
    public void setResting(boolean resting) {
        this.resting = resting;
    }
    public boolean isResting() {
        return resting;
    }
    public boolean isHoodLocked() {
        return hoodLocked;
    }
    public void setHoodLocked(boolean locked) {
        hoodLocked = locked;
    }
    public double goalDist() {
        return goalDist;
    }
}