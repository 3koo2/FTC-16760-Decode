package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.OpmodeConstants;
import org.firstinspires.ftc.teamcode.subsystems.constants.LimelightConstants;

import java.util.List;

public class LimelightSubsystem {
    Limelight3A limelight;
    Telemetry telemetry;
    GoBildaPinpointDriver pinpoint;
    Pose3D recentPose;

    public LimelightSubsystem(HardwareMap hardwareMap, Telemetry telemetry, GoBildaPinpointDriver pinpoint) {
        this.limelight = hardwareMap.get(Limelight3A.class, OpmodeConstants.LIMELIGHT_NAME);
        this.telemetry = telemetry;
        telemetry.setMsTransmissionInterval(LimelightConstants.TRANSMISSION_INTERVAL);
        limelight.pipelineSwitch(LimelightConstants.DEFAULT_PIPELINE);
        limelight.start();
        this.pinpoint = pinpoint;
        // TODO: pinpoint will likely be needed in some other class, so it should (maybe)...
        // be created in the superstructure and then accessed by superstructure.whatever
        // idk if we'd make pinpoint in superstructure or a pedro subsystem.
        // can we pass *this* to a function (ig its actually a constructor) from within a constructor.
        // probably i actually don't see why we couldn't. It doesn't matter tho, maybe we even create pinpoint and actually
        // pass the limelight subsystem the GOBildaPinponitControllller... >??? probabloy this rather
        // than passing it a subsystme and having to like get it or something.
    }

    public LLResultTypes.FiducialResult getTagId() {
        limelight.pipelineSwitch(LimelightConstants.APRILTAG_PIPELINE);
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        if (fiducialResults.isEmpty()) return null;
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            return fr;
        }
        return null;
    }

    public Pose2D getAvgPose() {
        limelight.updateRobotOrientation(pinpoint.getHeading(AngleUnit.DEGREES));
        LLResult res = limelight.getLatestResult();
        if (res == null || !res.isValid()) return pinpoint.getPosition();
        Pose3D mt2pose = res.getBotpose_MT2();
        if (mt2pose == null) return pinpoint.getPosition();
        double llWeight = (1.0 / pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES)) * (1.0 / limelight.getTimeSinceLastUpdate());
        double pinpointWeight = 1.0 - llWeight;
        return new Pose2D(
                DistanceUnit.INCH,
                (mt2pose.getPosition().x * llWeight) + (pinpoint.getPosition().getX(DistanceUnit.INCH) * pinpointWeight),
                (mt2pose.getPosition().y * llWeight) + (pinpoint.getPosition().getY(DistanceUnit.INCH) * pinpointWeight),
                AngleUnit.DEGREES,
                (mt2pose.getOrientation().getYaw(AngleUnit.DEGREES) * llWeight) + (pinpoint.getHeading(AngleUnit.DEGREES) * pinpointWeight)
        );
    }

    public Pose3D getLimelightOnlyPose(double yaw){
        limelight.updateRobotOrientation(yaw);

        LLResult res = limelight.getLatestResult();

        Pose3D megatag2 = res.getBotpose_MT2();

        return megatag2;
    }

    //  Should this really be like this for the subsystem? I feel like it would be nice if we just
    //  had something to coordinate between the turret and the limelight?
    //  Either returns the offset or -361 if it didn't find the tag
    public double getTurretAngleOffset(boolean teamRed) {
        limelight.pipelineSwitch(LimelightConstants.APRILTAG_PIPELINE);
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return -361;
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        if (fiducialResults.isEmpty()) return -361;
        LLResultTypes.FiducialResult tagFiducial = null;
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            int id = fr.getFiducialId();
            if (teamRed && id != 24) continue;
            if (id != 20) continue;
            tagFiducial = fr;
            break;
        }
        if (tagFiducial == null) return -361;
        double offset = tagFiducial.getTargetXDegrees();
        return offset - LimelightConstants.TURRET_ANGLE_OFFSET;
    }
}
