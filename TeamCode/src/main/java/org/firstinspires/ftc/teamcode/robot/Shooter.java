package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utils.Regression;

@Config
public class Shooter {
    public static class Params {
        public double farDist = 136;
        public double nearDist = 75;
        public double midDist = (farDist + nearDist) / 2;
        public double initShootPowerFar = 0.64;
        public double initShootPowerNear = 0.44;

        public double initHoodPosFar = 0.03;
        public double initHoodPosNear = 0.3814;
        public double autoHoodIncFar = 0.03, autoHoodIncNear = 0.03;

        public double shooterVelocityFar = 385;
        public double shooterVelocityNear = 200;
        public int minHoodPwm = 1440, maxHoodPwm = 1010;
        public double hoodFarRegressM = -0.004, hoodFarRegressB = 1.58;
        public double hoodNearRegressM = 0.1, hoodNearRegressB = 0.1;
        public double manualHoodInc = 0.01, manualShooterInc = 5;
    }
    public static Params params = new Params();
    private final Robot robot;
    private boolean shouldShoot;
    private final DcMotorEx motor1;
    private final DcMotorEx motor2;

    private final ServoImplEx hoodServo;

    // NL and NR signify the left and right near positions
    private double targetMotorPower;
    private double targetMotorVel;
    private double targetHoodPos;
    private double motorVelOffset, hoodOffset;
    private Regression hoodFarRegress, hoodNearRegress; // regressions output desired hood position as function of motor velocity (in degrees/s)
    private double totalTime;

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
        setShootingParams(params.initShootPowerFar, targetHoodPos);
        targetMotorVel = params.shooterVelocityFar;
        totalTime = 0;
    }

    public void update(double totalTime) {
        this.totalTime = totalTime;
        if(robot.g1.isFirstY())
            shouldShoot = !shouldShoot;


        double dist = distance();
        // automatically setting shooter hood and power
        if (dist > params.midDist) {
            targetMotorVel = params.shooterVelocityFar + motorVelOffset;
            targetHoodPos = Range.clip(hoodFarRegress.f(getShooterVelocity()) + hoodOffset, 0, 1);
        }
        else {
            targetMotorVel = params.shooterVelocityNear + motorVelOffset;
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

        // setting shooter power
        if(robot.g1.isFirstDpadRight()) {
            motorVelOffset += params.manualShooterInc;
        }
        else if(robot.g1.isFirstDpadLeft()) {
            motorVelOffset -= params.manualShooterInc;
        }

        if(shouldShoot)
            setShooterVel(targetMotorVel);
        else
            setShooterVel(0);
        hoodServo.setPosition(targetHoodPos);
    }
    public double getHoodPos() {
        return hoodServo.getPosition();
    }
    private void setShooterVel(double vel) {
        motor1.setVelocity(vel, AngleUnit.DEGREES);
        motor2.setVelocity(-vel, AngleUnit.DEGREES);
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
    private void setShootingParams(double power, double position) {
        targetMotorPower = power;
        targetHoodPos = position;
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