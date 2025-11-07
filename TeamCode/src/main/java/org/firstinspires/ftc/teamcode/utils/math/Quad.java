package org.firstinspires.ftc.teamcode.utils.math;

public class Quad extends Func {
    public final double a, b, c;
    public Quad(double a, double b, double c) {
        this.a = a;
        this.b = b;
        this.c = c;
    }
    @Override
    public double eval(double x) {
        return a * x * x + b * x + c;
    }
}
