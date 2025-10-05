package org.firstinspires.ftc.teamcode.utils;

public class Environment {
    public final double goalX, goalY;
    private ColorSensorBall.BallColor[] processedPattern;
    public Environment(double goalX, double goalY) {
        this.goalX = goalX;
        this.goalY = goalY;
        processedPattern = new ColorSensorBall.BallColor[3];
    }
    public void setProcessedPattern(ColorSensorBall.BallColor[] processedPattern) {
        this.processedPattern = processedPattern;
    }
    public ColorSensorBall.BallColor[] getProcessedPattern() {
        return processedPattern;
    }
}
