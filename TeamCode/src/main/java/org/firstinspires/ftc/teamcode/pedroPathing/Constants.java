package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static boolean secondaryPID = false; //use a secondary PID or not. See https://pedropathing.com/docs/pathing/tuning/pids
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5.35) //mass in kg
            .useSecondaryTranslationalPIDF(secondaryPID)
            .useSecondaryHeadingPIDF(secondaryPID)
            .useSecondaryDrivePIDF(secondaryPID)
            ;
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf") //replace with motor names, but this should probably be ok.
            .rightRearMotorName("rr")
            .leftRearMotorName("lr")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE) //make sure to reverse the correct side
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(66.14281361497294)
            .yVelocity(63.15601258765994)
            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-5) //offset from center (see pedropathing site)
            .strafePodX(0.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint") //replace with actual name, should be fine
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD) //probably what we use
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD) // TODO: TUNE WITH LOCALIZATION TEST
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);// TODO: TUNE WITH LOCALIZATION TEST
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
