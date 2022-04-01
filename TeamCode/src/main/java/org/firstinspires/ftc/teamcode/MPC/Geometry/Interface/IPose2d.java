package org.firstinspires.ftc.teamcode.MPC.Geometry.Interface;

import org.firstinspires.ftc.teamcode.MPC.Geometry.Pose2d;

public interface IPose2d<S> extends IRotation2d<S>, ITranslation2d<S> {
    Pose2d getPose();

    S transformBy(Pose2d transform);

    S mirror();
}
