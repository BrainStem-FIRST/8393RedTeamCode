package org.firstinspires.ftc.teamcode.utils.math;

public class Line extends Func {
    public final double m, b;
    public Line(double m, double b) {
        this.m = m;
        this.b = b;
    }
    @Override
    public double eval(double x) {
        return m * x + b;
    }
}
