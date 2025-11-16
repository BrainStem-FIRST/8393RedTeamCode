package org.firstinspires.ftc.teamcode.utils.math;

import androidx.annotation.NonNull;

public class Vector3 {
    public double x, y, z;
    public Vector3(double x, double y, double z) {
        this.x =x;
        this.y = y;
        this.z = z;
    }
    public Vector3 add(Vector3 v2) {
        return new Vector3(x + v2.x, y + v2.y, z + v2.z);
    }
    public Vector3 sub(Vector3 v2) {
        return new Vector3(x - v2.x, y - v2.y, z - v2.z);
    }
    public Vector3 mult(double s) {
        return new Vector3(x * s, y * s, z * s);
    }
    public void set(Vector3 v) {
        this.x = v.x;
        this.y = v.y;
        this.z = v.z;
    }
    public void set(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    @NonNull
    @Override
    public String toString() {
        return Math.floor(x * 100) / 100 + ", " + Math.floor(y * 100) / 100 + ", " + Math.floor(z * 100) / 100;
    }
    public Vector2 to2D() {
        return new Vector2(x, y);
    }
}
