package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.ColorSensorBall;

@Config
public class RGBLight {
    private static double inverseLerp(double a, double b, double x) {
        return (x - a) / (b - a);
    }
    // x = min + (max - min) * t
    // t = (x - min)/(max-min)
    public static class Params {
        public double minPwm = 1000, maxPwm = 2000;
        public double off = 0.01;
        public double red = 0.11, orange = 0.16, yellow = 0.21, green = 0.45, blue = 0.61, purple = 0.84, white = 0.91;
        public double flashTime = 1.4;
    }
    public static Params params = new Params();
    public final Robot robot;
    private final ServoImplEx light;
    private double lightValue;
    public enum LightState {
        OFF,
        SET,
        FLASH
    }
    private LightState lightState;
    private final ElapsedTime flashTimer, stateTimer;
    private double maxStateTime;
    public RGBLight(Robot robot) {
        this.robot = robot;
        light = robot.hardwareMap.get(ServoImplEx.class, "rgbLight");
        light.setPwmRange(new PwmControl.PwmRange(params.minPwm, params.maxPwm));
        lightState = LightState.SET;
        lightValue = 0;
        flashTimer = new ElapsedTime();
        stateTimer = new ElapsedTime();
        maxStateTime = -1;
    }
    public void update() {
        if(maxStateTime > 0) {
            if(stateTimer.seconds() > maxStateTime) {
                lightState = LightState.SET;
                lightValue = params.off;
                maxStateTime = -1;
            }
        }
        else if(!robot.shooter.isResting() && robot.shooter.getShooterVelocity() < robot.shooter.getMinShooterVel()) {
            lightState = LightState.SET;
            lightValue = params.red;
        }
        else if(robot.shooter.getShooterVelocity() > robot.shooter.getMaxShooterVel()) {
            lightValue = params.orange;
            lightState = LightState.SET;
        }
        else if(Math.abs(robot.getGoalHeading() - robot.getHeading()) < Robot.params.headingShootError) {
            lightState = LightState.SET;
            lightValue = params.blue;
        }

        else if(robot.isSlowTurn()) {
            lightState = LightState.SET;
            lightValue = params.yellow;
        }
        else {
            lightValue = params.off;
            lightState = LightState.SET;
        }
        updateLight();
    }
    private void updateLight() {
        double light = lightValue;
        if (lightState == LightState.FLASH) {
            if (flashTimer.seconds() > params.flashTime)
                light = params.off;
            if (flashTimer.seconds() > params.flashTime * 2)
                flashTimer.reset();
        }
        this.light.setPosition(light);
    }
    public double getLightValue() {
        return lightValue;
    }
    public void setState(LightState state, double color, double time) {
        lightState = state;
        flashTimer.reset();
        stateTimer.reset();
        maxStateTime = time;
        lightValue = color;
    }
}
