package org.firstinspires.ftc.teamcode.utils;

public class Regression {
    public final double m, b;
    public Regression(double m, double b) {
        this.m = m;
        this.b = b;
    }
    public double f(double x) {
        return m * x + b;
    }
}
