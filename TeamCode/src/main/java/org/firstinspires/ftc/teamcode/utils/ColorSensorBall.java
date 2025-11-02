package org.firstinspires.ftc.teamcode.utils;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.robot.Robot;

// has caching capability
public class ColorSensorBall {
    public static double[] normalGreenBallLow = {.095, .47, .34, 200};
    public static double[] normalGreenBallHigh = {.18, .52, .38, 10000};
    public static double[] normalPurpleBallLow = {.20, .26, .42, 200};
    public static double[] normalPurpleBallHigh = {.26, .31, .48, 10000};


    private double[] greenBallLow, greenBallHigh, purpleBallLow, purpleBallHigh;

    public enum BallColor {
        P, // purple
        G, // green
        N // none
    }

    private final ColorSensor colorSensor;
    private boolean updated;
    private double r, g, b;
    public ColorSensorBall(Robot robot, String name) {
        colorSensor = robot.hardwareMap.get(ColorSensor.class, name);
        updated = false;
        greenBallLow = normalGreenBallLow;
        greenBallHigh = normalGreenBallHigh;
        purpleBallLow = normalPurpleBallLow;
        purpleBallHigh = normalPurpleBallHigh;
    }

    public void update() {
        updated = false;
    }
    public BallColor getBallColor() {
        if(inBounds(greenBallLow[0], getRPercent(), greenBallHigh[0])
                && inBounds(greenBallLow[1], getGPercent(), greenBallHigh[1])
                && inBounds(greenBallLow[2], getBPercent(), greenBallHigh[2])
                && inBounds(greenBallLow[3], getSum(), greenBallHigh[3]))
            return BallColor.G;

        if(inBounds(purpleBallLow[0], getRPercent(), purpleBallHigh[0])
                && inBounds(purpleBallLow[1], getGPercent(), purpleBallHigh[1])
                && inBounds(purpleBallLow[2], getBPercent(), purpleBallHigh[2])
                && inBounds(purpleBallLow[3], getSum(), purpleBallHigh[3]))
            return BallColor.P;

        return BallColor.N;
    }

    private void updateChannels() {
        r = colorSensor.red();
        g = colorSensor.green();
        b = colorSensor.blue();

        updated = true;
    }
    private boolean inBounds(double low, double val, double high) {
        return val >= low && val <= high;
    }

    private double getRPercent() {
        if(!updated)
            updateChannels();
        return r / getSum();
    }
    private double getGPercent() {
        if(!updated)
            updateChannels();
        return g / getSum();
    }
    private double getBPercent() {
        if(!updated)
            updateChannels();
        return b / getSum();
    }
    private double getSum() {
        if(!updated)
            updateChannels();
        return r + g + b;
    }
    public String getRGBS() {
        return getRPercent() + ", " + getGPercent() + ", " + getBPercent() + ", " + getSum();
    }
}
