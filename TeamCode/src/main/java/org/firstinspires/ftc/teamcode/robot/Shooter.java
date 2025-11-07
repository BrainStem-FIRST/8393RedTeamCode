package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.math.Cubic;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.math.Line;
import org.firstinspires.ftc.teamcode.utils.math.Quad;

@Config
public class Shooter {
    public static class Params {
        public double kP = 0.02, kI = 0, kD = 0.003, maxShooterPower = 0.8, startupPower = 1;
        public double startupDist = 50; // if velocity is less than target by this amount, i only apply startup power
        public double farDistTele = 151.7, farDistAuto = 145; // far dist auto is guessed
        public double nearDist = 65;
        public double midDist = (farDistTele + nearDist) / 2;

        public double shooterVelocityFarTele = 400, shooterPowerFarTele = 0.62;
        public double shooterVelocityNear = 285, shooterPowerNear = 0.45;

        public double minShooterVelFarTele = 360, maxShooterVelFarTele = 410;
        public double minShooterVelNear = 260, maxShooterVelNear = 290;
        public double restPowerOffset = 1;
        public int minHoodPwm = 1800, maxHoodPwm = 1400;

//        public double hoodFarA = 0, hoodFarB = 0, hoodFarC = 0, hoodFarD = 0; //y=-0.000007323075x^{3}+0.008641343x^{2}-3.401257x+446.75658
        public double hoodNearM = -0.0113448, hoodNearB = 3.64514; //y=-0.0113448x+3.61914
        public double hoodFarM = -0.00223382, hoodFarB = 0.900779; //y=-0.00223382x+0.900779

        public double manualHoodInc = 0.01, manualShooterInc = 2.5;
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
    private final Line hoodFarRegress, hoodNearRegress; // regressions output desired hood position as function of motor velocity (in degrees/s)
    private int zone;
    private double shooterPower, prevShooterPower, prevHoodPos;
    private boolean resting, hoodLocked;
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

        hoodFarRegress = new Line(params.hoodFarM, params.hoodFarB);
        hoodNearRegress = new Line(params.hoodNearM, params.hoodNearB);

        targetMotorPower = 0;
        targetHoodPos = 1;
        targetMotorVel = params.shooterVelocityNear;
        zone = 0;

        shooterPid = new PIDController(params.kP, params.kI, params.kD);
        shooterPid.setOutputBounds(-params.maxShooterPower, params.maxShooterPower);
        shooterPid.setTarget(targetMotorVel);

        hoodOffset = params.hoodOffset;
        resting = true;
    }
    public void resetCaches() {
        powerUpdated = false;
        velUpdated = false;
    }
    public void update() {
        if(robot.g1.isFirstY())
            resting = !resting;
        // zone changers
        if(robot.g1.isFirstDpadUp()) {
            zone = 0;
            shooterPid.setTarget(targetMotorVel);
        }
        else if(robot.g1.isFirstDpadDown()) {
            zone = 1;
            shooterPid.setTarget(targetMotorVel);
        }

        // automatically setting shooter hood and power
        if(zone == 0) {
            targetMotorVel = params.shooterVelocityNear;
            minMotorVel = params.minShooterVelNear;
            maxMotorVel = params.maxShooterVelNear;
            targetMotorPower = params.shooterPowerNear;
            if(!hoodLocked)
                targetHoodPos = Range.clip(Range.clip(hoodNearRegress.eval(getShooterVelocity()), 0, 1) + hoodOffset, 0, 1);
        }
        else {
            targetMotorVel = params.shooterVelocityFarTele;
            minMotorVel = params.minShooterVelFarTele;
            maxMotorVel = params.maxShooterVelFarTele;
            targetMotorPower = params.shooterPowerFarTele;
            if(!hoodLocked)
                targetHoodPos = Range.clip(Range.clip(hoodFarRegress.eval(getShooterVelocity()), 0, 1) + hoodOffset, 0, 1);
        }

        // setting hood position
        if(robot.g2.isFirstDpadUp())
            hoodOffset += params.manualHoodInc;
        else if(robot.g2.isFirstDpadDown())
            hoodOffset -= params.manualHoodInc;

        // setting shooter power
        if(robot.g2.isFirstDpadRight()) {
            targetMotorVel += params.manualShooterInc;
            shooterPid.setTarget(targetMotorVel);
        }
        else if(robot.g2.isFirstDpadLeft()) {
            targetMotorVel -= params.manualShooterInc;
            shooterPid.setTarget(targetMotorVel);
        }
        // do not adjust after shoot button has been pressed
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
                robot.telemetry.addData("shooter power", shooterPower);
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
}