package org.firstinspires.ftc.teamcode.utils.math;

public class Cos extends Func {
    public final double a, b, c;
    public Cos(double a, double b, double c) {
        this.a = a;
        this. b = b;
        this.c = c;
    }
    @Override
    public double eval(double x) {
        return a * Math.cos(b * x) + c;
    }
}
