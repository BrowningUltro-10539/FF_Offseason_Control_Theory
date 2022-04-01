package org.firstinspires.ftc.teamcode.MPC.Utils;

public interface Interpolable<T> {
    T interpolate(T other, double x);
}
