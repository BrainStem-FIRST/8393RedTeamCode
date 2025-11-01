package org.firstinspires.ftc.teamcode.utils;

public class ExpoRegression {
    public final double a, b;
    public ExpoRegression(double a, double b) {
        this.a = a;
        this.b = b;
    }
    public double f(double x) {
        return a * Math.pow(b , x);
    }
}
