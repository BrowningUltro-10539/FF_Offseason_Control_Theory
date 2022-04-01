package org.firstinspires.ftc.teamcode.MPC.Geometry.Interface;

import org.firstinspires.ftc.teamcode.MPC.Geometry.State;

public interface ICurvature<S> extends State<S> {
    double getCurvature();

    double getDCurvatureDs();
}
