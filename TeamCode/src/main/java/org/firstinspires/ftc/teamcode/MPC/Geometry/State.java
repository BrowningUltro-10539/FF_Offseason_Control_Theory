package org.firstinspires.ftc.teamcode.MPC.Geometry;

import org.firstinspires.ftc.teamcode.MPC.Utils.Interpolable;

public interface State<S> extends Interpolable<S> {
    double distance(final S other);
    boolean equals(final Object other);
    String toString();
}
