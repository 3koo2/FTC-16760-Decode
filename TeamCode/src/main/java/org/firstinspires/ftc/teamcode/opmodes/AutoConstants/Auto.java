package org.firstinspires.ftc.teamcode.opmodes.AutoConstants;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.lib.PathConstant;

import java.util.Dictionary;
import java.util.Hashtable;

public class Auto implements PathConstant {
    public static Pose start = new Pose(0, 0, Math.toRadians(0));
    public static Pose firstShoot = new Pose(-1, -1, Math.toRadians(45));
    public static Pose GPPStart = new Pose(-5, -20, Math.toRadians(90));
    public static Pose GPPEnd = new Pose(0, -20, Math.toRadians(90));
    public static Pose PGPStart = new Pose(-5, -15, Math.toRadians(90));
    public static Pose PGPEnd = new Pose(0, -15, Math.toRadians(90));
    public static Pose PPGStart = new Pose(-5, -10, Math.toRadians(90));
    public static Pose PPGEnd = new Pose(0, -10, Math.toRadians(90));
    public static Pose secondShoot = new Pose(-1, -1, Math.toRadians(45));
    Follower follower;

    public Auto(Follower follower){
        this.follower = follower;
    }

    public Dictionary<String, PathChain> buildPaths() {
        Hashtable<String, PathChain> hashtable = new Hashtable<String, PathChain>();

        PathChain startToFirst = follower.pathBuilder()
                .addPath(new BezierLine(start, firstShoot))
                .setLinearHeadingInterpolation(start.getHeading(), firstShoot.getHeading())
                .build();
        hashtable.put("Drive To First", startToFirst);

        return hashtable;
    }
}
