package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.ColorSensorBall;
import org.firstinspires.ftc.teamcode.utils.PIDController;

public class Shooter {

    private final Robot robot;

    // OFF: motor is unmoving
    // SHOOTING: motor is running to shoot the balls from the indexer
    public enum State {
        OFF, SHOOTING
    }

    private State state;
    private final DcMotorEx motor;
    private double motorPower;

    public Shooter(Robot robot) {
        this.robot = robot;
        state = Shooter.State.OFF;
        motor = robot.hardwareMap.get(DcMotorEx.class, "intakeIndexer");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorPower = 0;
    }

    public void update() {
        switch(state){
            case OFF:
                break; //TODO: fill in yippee
            case SHOOTING:
                break; //TODO: fill in la la la
        }
    }

}
