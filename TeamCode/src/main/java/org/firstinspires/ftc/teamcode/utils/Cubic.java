package org.firstinspires.ftc.teamcode.utils;

public class Cubic {
    public final double a, b, c, d;
    public Cubic(double a, double b, double c, double d) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
    }
    public double f(double x) {
        return a * x * x * x + b * x * x + c * x + d;
    }
}
