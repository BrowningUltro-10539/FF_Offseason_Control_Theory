package org.firstinspires.ftc.teamcode.MPC.Geometry.Interface;

import org.firstinspires.ftc.teamcode.MPC.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.MPC.Geometry.State;

public interface IRotation2d<S> extends State<S> {
    Rotation2d getRotation();
}
