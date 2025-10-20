package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Shooter {
    public static final double triggerPressThreshold = 0.1;
    public static final double shootPowerFar = 0.725;
    public static final double shootPowerNL = 0.6; //TODO: what works best here?
    public static final double shootPowerNR = 0.5; //TODO: ?

    public static final double hoodPositionFar = 0.0; //TODO: ?
    public static final double hoodPositionNL = 0.0; //TODO: ?
    public static final double hoodPositionNR = 0.0; //TODO: ?

    public static double minVelPowerCorrespondence = 0.9;
    public static int minHoodPwm = 1440, maxHoodPwm = 1010;
    public static double manualHoodInc = 0.02, manualShooterInc = 0.02;
    private final Robot robot;

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
    }

    // TODO: each position has a different speed and hood position
    public void update() {
        // setting hood position
        if(robot.g1.isFirstDpadUp())
            hoodServo.setPosition(getHoodPos() + manualHoodInc);
        else if(robot.g1.isFirstDpadDown())
            hoodServo.setPosition(getHoodPos() - manualHoodInc);

        // setting shooter power
        if(robot.g1.isFirstY())
            motorPower += manualShooterInc;
        else if(robot.g1.isFirstA())
            motorPower -= manualShooterInc;
        else if(robot.g1.isFirstX())
            motorPower = 0;

//        // if (robot x is some measurement past goalX && robot y is very different from goalY)
//            setStateShooting(shootPowerFar, hoodPositionFar);
//        // else if (robot x is some measurement past goalX && robot y is closeish to goalY)
//            setStateShooting(shootPowerNL, hoodPositionNL);
//        // else if (robot x is more than the past measurements past goalX && robot y is closeish to goalY)
//            setStateShooting(shootPowerNR, hoodPositionNR);

        setShooterPower(motorPower);
    }
    public double getHoodPos() {
        return hoodServo.getPosition();
    }

    private void setStateShooting(double power, double position){
        state = State.SHOOTING;
        motorPower = power;
    }
    private boolean shootTriggerPressed() { //TODO: maybe delete? since it's always spinning
        // checking in-taking
        return robot.g1.gamepad.right_trigger > triggerPressThreshold;
    }

    private void setShooterPower(double power) {
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

    public void _setShooterPower(double power) {
        motor1.setPower(power);
        motor2.setPower(-power);
    }
    public double getShooterPower() {
        return motor1.getPower();
    }

}