package org.firstinspires.ftc.teamcode.utils;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.robot.Robot;

// has caching capability
public class ColorSensorBall {
    public static int[] greenBallLow = {0, 0, 0};
    public static int[] greenBallHigh = {20, 20, 20};

    public static int[] purpleBallLow = {0, 0, 0};
    public static int[] purpleBallHigh = {20, 20, 20};

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
        if(inBounds(greenBallLow[0], getR(), greenBallHigh[0])
                && inBounds(greenBallLow[1], getG(), greenBallHigh[1])
                && inBounds(greenBallLow[2], getB(), greenBallHigh[2]))
            return BallColor.GREEN;

        if(inBounds(purpleBallLow[0], getR(), purpleBallHigh[0])
                && inBounds(purpleBallLow[1], getG(), purpleBallHigh[1])
                && inBounds(purpleBallLow[2], getB(), purpleBallHigh[2]))
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
    private boolean inBounds(int low, int val, int high) {
        return val >= low && val <= high;
    }

    private int getR() {
        if(!updated)
            updateChannels();
        return r;
    }
    private int getG() {
        if(!updated)
            updateChannels();
        return g;
    }
    private int getB() {
        if(!updated)
            updateChannels();
        return b;
    }
}
