package org.firstinspires.ftc.teamcode.MPC.Geometry.Interface;

import org.firstinspires.ftc.teamcode.MPC.Geometry.State;
import org.firstinspires.ftc.teamcode.MPC.Geometry.Translation2d;

public interface ITranslation2d<S> extends State<S> {
    Translation2d getTranslation();
}
