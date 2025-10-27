package org.firstinspires.ftc.teamcode.utils;

public class QuadRegression {
    public final double a, b, c;
    public QuadRegression(double a, double b, double c) {
        this.a = a;
        this.b = b;
        this.c = c;
    }
    public double f(double x) {
        return a * x * x + b * x + c;
    }
}
