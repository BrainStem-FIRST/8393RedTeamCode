package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Shooter {
    public static double farDist = 136;
    public static double nearDist = 75;
    public static double midDist = (farDist+nearDist)/2;
    public static double shooterSetupTime = 10, shooterPowerAutoAdjust = 0.02, shooterPowerAdjustError = 20;
    public static final double triggerPressThreshold = 0.1;
    public static double shootPowerFar = 0.62;
    public static double shootPowerNear = 0.44;

    public static double hoodPositionFar = 0.03;
    public static double hoodPositionNear = 0.3814;

    public static final double shooterVelocityFar = 1520;
    public static final double shooterVelocityNear = 1220;
    public static int minHoodPwm = 1440, maxHoodPwm = 1010;
    public static double manualHoodInc = 0.01, manualShooterInc = 0.01;
    private final Robot robot;
    private boolean shouldShoot;

    // SHOOTING: motor is running to shoot the balls from the indexer
    public enum State {
        SHOOTING
    }

    private State state;
    private final DcMotorEx motor1;
    private final DcMotorEx motor2;

    private final ServoImplEx hoodServo;

    // NL and NR signify the left and right near positions
    private double motorPower;
    private double targetMotorVel, targetMotorPower;
    private double totalTime;

    public Shooter(Robot robot) {
        this.robot = robot;
        state = Shooter.State.SHOOTING;

        motor1 = robot.hardwareMap.get(DcMotorEx.class, "shooter1");
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor2 = robot.hardwareMap.get(DcMotorEx.class, "shooter2");
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hoodServo = robot.hardwareMap.get(ServoImplEx.class, "hoodServo");
        hoodServo.setPwmRange(new PwmControl.PwmRange(minHoodPwm, maxHoodPwm));

        motorPower = 0;
        setStateShooting(shootPowerFar, hoodPositionFar);
        targetMotorVel = shooterVelocityFar;
        totalTime = 0;
    }

    // TODO: each position has a different speed and hood position
    public void update(double totalTime) {
        this.totalTime = totalTime;
        if(robot.g1.isFirstY())
            shouldShoot = !shouldShoot;

        if(!shouldShoot) {
            setShooterPower(0);
            return;
        }
        double dist = distance();



        // manually setting hood position
        if(robot.g1.isFirstDpadUp()) {
            if (dist > midDist) {
                hoodPositionFar += manualHoodInc;
                hoodServo.setPosition(hoodPositionFar);
            }
            else {
                hoodPositionNear += manualHoodInc;
                hoodServo.setPosition(hoodPositionNear);
            }
        }
        else if(robot.g1.isFirstDpadDown()) {
            if (dist > midDist)
                hoodPositionFar -= manualShooterInc;
            else
                hoodPositionNear -= manualShooterInc;
        }

        // manually setting shooter power
        if(robot.g1.isFirstDpadRight() || robot.g2.isFirstY()) {
            if(dist > midDist)
                shootPowerFar += manualShooterInc;
            else
                shootPowerNear += manualShooterInc;
        }
        else if(robot.g1.isFirstDpadLeft() || robot.g2.isFirstA()) {
            if(dist > midDist)
                shootPowerFar -= manualShooterInc;
            else
                shootPowerNear -= manualShooterInc;
        }
        else if(robot.g2.isFirstBack())
            motorPower = 0;

        // automatically setting shooter hood and power
        if (dist < midDist) {
            targetMotorVel = shooterVelocityNear;
            setStateShooting(shootPowerNear, hoodPositionNear);
        }
        else {
            targetMotorVel = shooterVelocityFar;
            setStateShooting(shootPowerFar, hoodPositionFar);
        }

        setShooterPower(motorPower);
    }
    public double getHoodPos() {
        return hoodServo.getPosition();
    }

    public double distance(){
        return Math.sqrt(Math.pow(robot.follower.getPose().getX()-robot.goalX, 2) + Math.pow(robot.follower.getPose().getY()-robot.goalY, 2));
    }
    private void setStateShooting(double power, double position){
        state = State.SHOOTING;
        targetMotorPower = power;
        motorPower = power;
        hoodServo.setPosition(position);
    }

    private void setShooterPower(double power) {
        if (totalTime > shooterSetupTime && Math.abs(getShooterVelocity() - targetMotorVel) > shooterPowerAdjustError) {
            if (getShooterVelocity() < targetMotorVel && motorPower - targetMotorPower < 0.02) {
                motorPower += shooterPowerAutoAdjust;
            }
            else if(getShooterVelocity() > targetMotorVel && targetMotorPower - motorPower < 0.02)
                motorPower -= shooterPowerAutoAdjust;
        }
        motor1.setPower(power);
        motor2.setPower(-power);
    }


    private int getMotor1Pos() {
        return motor1.getCurrentPosition();
    }
    private int getMotor2Pos() {
        return motor2.getCurrentPosition();
    }
    public State getState() {
        return state;
    }

    public double getShooterVelocity() {
        return motor1.getVelocity();
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
    public double getShooterMiliAmps() {
        return motor1.getCurrent(CurrentUnit.MILLIAMPS);
    }
}