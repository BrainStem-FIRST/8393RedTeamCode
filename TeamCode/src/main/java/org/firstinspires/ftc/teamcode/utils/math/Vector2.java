package org.firstinspires.ftc.teamcode.utils.math;

import androidx.annotation.NonNull;

public class Vector2 {
    public double x, y;
    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public Vector2 add(Vector2 v2) {
        return new Vector2(x + v2.x, y + v2.y);
    }
    public Vector2 sub(Vector2 v2) {
        return new Vector2(x - v2.x, y - v2.y);
    }
    public Vector2 mult(double s) {
        return new Vector2(x * s, y * s);
    }
    @NonNull
    @Override
    public String toString() {
        return Math.floor(x * 100)/100 + ", " + Math.floor(y * 100)/100;
    }
    public void set(Vector2 v) {
        this.x = v.x;
        this.y = v.y;
    }
}
