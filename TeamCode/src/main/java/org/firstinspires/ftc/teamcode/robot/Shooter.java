package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.ExpoRegression;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.QuadRegression;

@Config
public class Shooter {
    public static class Params {
        public double kP = 0.05, kI = 0, kD = 0.003, maxShooterPower = 0.8;
        public double farDist = 145;
        public double nearDist = 65;
        public double midDist = (farDist + nearDist) / 2;

        public double initHoodPosFar = 0.08;

        public double shooterVelocityFarTele = 400, shooterPowerFarTele = 0.655;
        public double shooterVelocityNear = 275, shooterPowerNear = 0.45;
        public double shooterVelocityFarAuto = 395, shooterPowerFarAuto = 0.655;
        public double t = 0.2;
        public int minHoodPwm = 1800, maxHoodPwm = 1400;
        //y=0.000203197x^{2}-0.16259x+32.54614
        public double hoodTeleFarRegressA = 0.000203197, hoodTeleFarRegressB = -0.16259, hoodTeleFarRegressC = 32.54614;
        //y=0.0009x^{2}-0.4767x+63.8595
        public double hoodNearRegressA = 0.0009, hoodNearRegressB = -0.4767, hoodNearRegressC = 63.8595;
        public double hoodAutoFarRegressA = 2.21031 * Math.pow(10, 14), hoodAutoFarRegressB = 0.911603;
        // y=(2.21031*10^14) * 0.911603^x
        public double manualHoodInc = 0.01, manualShooterInc = 5;
    }
    public static Params params = new Params();
    private final Robot robot;
    private boolean shouldShoot;
    private final DcMotorEx motor1;
    private final DcMotorEx motor2;

    private final ServoImplEx hoodServo;
    private final PIDController shooterPid;

    // NL and NR signify the left and right near positions
    private double targetMotorPower;
    private double targetMotorVel;
    private double targetHoodPos;
    private double motorVelOffset, hoodOffset;
    private final QuadRegression hoodTeleFarRegress, hoodNearRegress; // regressions output desired hood position as function of motor velocity (in degrees/s)
    private final ExpoRegression hoodAutoFarRegress;
    private int zone;
    private double shooterPower, prevShooterPower, prevHoodPos;
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

        hoodTeleFarRegress = new QuadRegression(params.hoodTeleFarRegressA, params.hoodTeleFarRegressB, params.hoodTeleFarRegressC);
        hoodNearRegress = new QuadRegression(params.hoodNearRegressA, params.hoodNearRegressB, params.hoodNearRegressC);
        hoodAutoFarRegress = new ExpoRegression(params.hoodAutoFarRegressA, params.hoodAutoFarRegressB);

        targetMotorPower = 0;
        targetHoodPos = params.initHoodPosFar;
        targetMotorVel = params.shooterVelocityFarTele;
        zone = 1;

        shooterPid = new PIDController(params.kP, params.kI, params.kD);
        shooterPid.setOutputBounds(-params.maxShooterPower, params.maxShooterPower);
        shooterPid.setTarget(targetMotorVel);
    }

    public void update() {
        if(robot.g1.isFirstY())
            shouldShoot = !shouldShoot;
        if(robot.g1.isFirstBack())
            zone = (zone + 1) % 3;

        double dist = distance();
        // automatically setting shooter hood and power
        if (zone == 0) {
            targetMotorVel = params.shooterVelocityFarTele + motorVelOffset;
            targetMotorPower = params.shooterPowerFarTele;
            shooterPid.setTarget(targetMotorVel);
            targetHoodPos = Range.clip(Range.clip(hoodTeleFarRegress.f(getShooterVelocity()), 0, 1) + hoodOffset, 0, 1);
        }
        else if(zone == 1) {
            targetMotorVel = params.shooterVelocityNear + motorVelOffset;
            targetMotorPower = params.shooterPowerNear;
            shooterPid.setTarget(targetMotorVel);
            targetHoodPos = Range.clip(Range.clip(hoodNearRegress.f(getShooterVelocity()), 0, 1) + hoodOffset, 0, 1);
        }
        else if(zone == 2) {
            targetMotorVel = params.shooterVelocityFarAuto + motorVelOffset;
            targetMotorPower = params.shooterPowerFarAuto;
            shooterPid.setTarget(targetMotorVel);
            targetHoodPos = Range.clip(Range.clip(hoodAutoFarRegress.f(getShooterVelocity()), 0, 1) + hoodOffset, 0, 1);
        }

        // setting hood position
        if(robot.g1.isFirstDpadUp())
            hoodOffset += params.manualHoodInc;
        else if(robot.g1.isFirstDpadDown())
            hoodOffset -= params.manualHoodInc;

        // setting shooter power
        if(robot.g1.isFirstDpadRight()) {
            motorVelOffset += params.manualShooterInc;
            shooterPid.setTarget(targetMotorVel + motorVelOffset);
        }
        else if(robot.g1.isFirstDpadLeft()) {
            motorVelOffset -= params.manualShooterInc;
            shooterPid.setTarget(targetMotorVel + motorVelOffset);
        }

        // do not adjust after shoot button has been pressed
        if(robot.transfer.getTransferState() != Transfer.TransferState.TRANSFERRING) {
            shooterPower = 0;
            if (shouldShoot) {
                // params.t is a value between 0 and 1 (I have it set to 0.9)
                shooterPower = shooterPid.update(getShooterVelocity()) * params.t + targetMotorPower * (1 - params.t);
            }

            if(shooterPower != prevShooterPower)
                setShooterPower(shooterPower);
            if(targetHoodPos != prevHoodPos)
                hoodServo.setPosition(targetHoodPos);
        }
        prevShooterPower = shooterPower;
        prevHoodPos = targetHoodPos;
    }
    public double getHoodPos() {
        return hoodServo.getPosition();
    }
    public int getZone() {
        return zone;
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
        return Math.sqrt(Math.pow(robot.follower.getPose().getX()-Robot.params.goalX, 2) + Math.pow(robot.follower.getPose().getY()-Robot.params.goalY, 2));
    }
    public double getShooterVelocity() {
        return (motor1.getVelocity(AngleUnit.DEGREES) - motor2.getVelocity(AngleUnit.DEGREES)) * 0.5;
    }
    public double getShooterPower() {
        return motor1.getPower();
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
}