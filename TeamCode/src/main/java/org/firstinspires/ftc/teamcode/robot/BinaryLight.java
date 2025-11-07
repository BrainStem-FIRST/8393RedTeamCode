package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class BinaryLight {
    private static double inverseLerp(double a, double b, double x) {
        return (x - a) / (b - a);
    }
    // x = min + (max - min) * t
    // t = (x - min)/(max-min)
    public static class Params {
        public double minPwm = 500, maxPwm = 2520;
        public double off = inverseLerp(minPwm, maxPwm, 600);
        public double bright = inverseLerp(minPwm, maxPwm, 1700);
        public double dim = inverseLerp(minPwm, maxPwm, 750);
        public double flashTime = 1.4;
    }
    public static Params params = new Params();
    public final Robot robot;
    private final ServoImplEx light;
    private double lightValue;
    public enum LightState {
        SET,
        FLASH_BRIGHT,
        FLASH_DIM
    }
    private LightState lightState;
    private final ElapsedTime flashTimer;
    public BinaryLight(Robot robot) {
        this.robot = robot;
        light = robot.hardwareMap.get(ServoImplEx.class, "light");
        light.setPwmRange(new PwmControl.PwmRange(params.minPwm, params.maxPwm));
        lightState = LightState.SET;
        lightValue = 0;
        flashTimer = new ElapsedTime();
    }
    public void update() {
        if(robot.shooter.getShooterVelocity() < robot.shooter.getMinShooterVel() && !robot.shooter.isResting()) {
            if(lightState != LightState.FLASH_BRIGHT)
                flashTimer.reset();
            lightState = LightState.FLASH_BRIGHT;
        }
        else {
            lightValue = params.off;
            lightState = LightState.SET;
        }
        updateLight();
    }
    private void updateLight() {

        if (lightState == LightState.FLASH_BRIGHT || lightState == LightState.FLASH_DIM) {
            double flashValue = params.bright;
            if(lightState == LightState.FLASH_DIM)
                flashValue = params.dim;


            if (flashTimer.seconds() > params.flashTime)
                lightValue = params.off;
            else
                lightValue = flashValue;
            if (flashTimer.seconds() > params.flashTime * 2)
                flashTimer.reset();
        }
        light.setPosition(lightValue);
    }
    private double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
    public double getLightValue() {
        return lightValue;
    }
    public void setLightState(LightState lightState) {
        this.lightState = lightState;
    }
    public void setLightValue(double value) {
        lightValue = value;
    }
}
