package org.firstinspires.ftc.teamcode.Auto;
// make sure this aligns with class location
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "auto")
public class auto extends OpMode {
    public static double DISTANCE = 40;
    private boolean forward = true;

    private Path forwards;
    private Path backwards;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(120.4173913043478, 119.16521739130434, Math.toRadians(45)));
    }

    /** This initializes the Follower and creates the forward and backward Paths. */
    @Override
    public void init_loop() {
        telemetryM.debug("This will activate all the PIDF(s)");
        telemetryM.debug("The robot will go forward and backward continuously along the path while correcting.");
        telemetryM.debug("You can adjust the PIDF values to tune the robot's drive PIDF(s).");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void start() {
        follower.activateAllPIDFs();
        forwards = new Path(new BezierLine(new Pose(120.4173913043478,119.16521739130434), new Pose(107.99130434782609,107.0086956521739)));
        forwards.setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(53));
        //backwards = new Path(new BezierLine(new Pose(DISTANCE + 72,72), new Pose(72,72)));
        ;
        //backwards.setConstantHeadingInterpolation(0);
        follower.followPath(forwards);
    }

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry */
    @Override
    public void loop() {
        follower.update();
        draw();

        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
               // follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }

        telemetryM.debug("Driving Forward?: " + forward);
        telemetryM.update(telemetry);
    }
}

