package org.firstinspires.ftc.teamcode.utils;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.robot.Robot;

// has caching capability
public class ColorSensorBall {
    public static int[] greenBallLow = {0, 0, 0, 0};
    public static int[] greenBallHigh = {1, 1, 1, 1};

    public static int[] purpleBallLow = {0, 0, 0, 0};
    public static int[] purpleBallHigh = {1, 1, 1, 1};

    public enum BallColor {
        PURPLE,
        GREEN,
        NONE
    }

    private final ColorSensor colorSensor;
    private boolean updated;
    private int r, g, b;
    public ColorSensorBall(Robot robot, String name) {
        colorSensor = robot.hardwareMap.get(ColorSensor.class, name);
        updated = false;
    }

    public void update() {
        updated = false;
    }
    public BallColor getBallColor() {
        if(inBounds(greenBallLow[0], getRPercent(), greenBallHigh[0])
                && inBounds(greenBallLow[1], getGPercent(), greenBallHigh[1])
                && inBounds(greenBallLow[2], getBPercent(), greenBallHigh[2])
                && inBounds(greenBallLow[3], getSum(), greenBallHigh[3]))
            return BallColor.GREEN;

        if(inBounds(purpleBallLow[0], getRPercent(), purpleBallHigh[0])
                && inBounds(purpleBallLow[1], getGPercent(), purpleBallHigh[1])
                && inBounds(purpleBallLow[2], getBPercent(), purpleBallHigh[2])
                && inBounds(purpleBallLow[3], getSum(), purpleBallHigh[3]))
            return BallColor.PURPLE;

        return BallColor.NONE;
    }

    private void updateChannels() {
        int sum = colorSensor.argb();
        r = Color.red(sum);
        g = Color.green(sum);
        b = Color.blue(sum);
        updated = true;
    }
    private boolean inBounds(double low, double val, double high) {
        return val >= low && val <= high;
    }

    private double getRPercent() {
        if(!updated)
            updateChannels();
        return r * 1. / (r + g + b);
    }
    private double getGPercent() {
        if(!updated)
            updateChannels();
        return g * 1. / (r + g + b);
    }
    private double getBPercent() {
        if(!updated)
            updateChannels();
        return b * 1. / (r + g + b);
    }
    private double getSum() {
        if(!updated)
            updateChannels();
        return r + g + b;
    }
}
