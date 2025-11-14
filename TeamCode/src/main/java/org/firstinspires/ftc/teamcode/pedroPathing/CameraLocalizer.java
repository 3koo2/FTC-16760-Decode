package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Objects;

public class CameraLocalizer implements Localizer {
    private final GoBildaPinpointDriver odo;
    private final Limelight3A ll;
    private double previousHeading;
    private double totalHeading;
    private Pose startPose;
    private Pose currentVelocity;
    private Pose calculatedPose;
    private final double maximumLLAngleVelo;
    private final double maximumLLUpdateTime;

    public CameraLocalizer(HardwareMap hwMap, CameraConstants constants){this(hwMap, constants, new Pose());}

    //Basically the same constructor as the pinpoint localizer but makes a limelight also and grabs its constants
    public CameraLocalizer(HardwareMap hwMap, CameraConstants constants, Pose setStartPose){
        odo = hwMap.get(GoBildaPinpointDriver.class, constants.hardwareMapName);
        setOffsets(constants.forwardPodY, constants.strafePodX, constants.distanceUnit);
        if(constants.yawScalar.isPresent()) {
            odo.setYawScalar(constants.yawScalar.getAsDouble());
        }
        if(constants.customEncoderResolution.isPresent()) {
            odo.setEncoderResolution(constants.customEncoderResolution.getAsDouble(), DistanceUnit.INCH);
        } else {
            odo.setEncoderResolution(constants.encoderResolution);
        }
        odo.setEncoderDirections(constants.forwardEncoderDirection, constants.strafeEncoderDirection);
        setStartPose(setStartPose);
        totalHeading = 0;
        calculatedPose = startPose;
        currentVelocity = new Pose();
        previousHeading = setStartPose.getHeading();

        maximumLLAngleVelo = constants.maxLimelightAngleVelocity;
        maximumLLUpdateTime = constants.maxLimelightUpdateTime;
        ll = hwMap.get(Limelight3A.class, constants.limelightName);
        ll.pipelineSwitch(constants.limelightPipeline);
        ll.start();

    }

    private void setOffsets(double xOffset, double yOffset, DistanceUnit unit) {
        odo.setOffsets(xOffset, yOffset, unit);
    }

    /**
     * This returns the current pose estimate from the Localizer.
     *
     * @return returns the pose as a Pose object.
     */
    @Override
    public Pose getPose() {
        return calculatedPose;
    }

    /**
     * This returns the current velocity estimate from the Localizer.
     *
     * @return returns the velocity as a Pose object.
     */
    @Override
    public Pose getVelocity() {
        return currentVelocity;
    }

    public Vector getVelocityVector() {
        return currentVelocity.getAsVector();
    }

    //I'm not sure about this function. It doesn't seem like megatag2 will let me tell it a pose
    //at all so this might not work with pedro. I just have it set the odometry pose for now
    @Override
    public void setPose(Pose setPose) {
        odo.setPosition(PoseConverter.poseToPose2D(setPose, PedroCoordinates.INSTANCE));
        calculatedPose = setPose;
        previousHeading = setPose.getHeading();
    }

//  Grabs stuff from ll/megatag if it's slow enough and recent enough, if not pose is estimated
//  off of pinpoint. I might do a thing that looks at current pose - previous pose over time so
//  we actually use mt2 for velocity. Maybe mt2 is good for just pose and pinpoint is good for motion
    @Override
    public void update() {
        ll.updateRobotOrientation(odo.getHeading(AngleUnit.DEGREES));
        LLResult res = ll.getLatestResult();
        if (res == null || !res.isValid()){
            updateOnPinpoint();
            return;
        }
        Pose3D mt2pose = res.getBotpose_MT2();
        if (mt2pose == null) {
            updateOnPinpoint();
            return;
        }
        if (odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES) > maximumLLAngleVelo || ll.getTimeSinceLastUpdate() > maximumLLUpdateTime){
            updateOnPinpoint();
            return;
        }
        Pose mt2poseGood = poseFromPose3d(mt2pose);
        totalHeading += MathFunctions.getSmallestAngleDifference(mt2poseGood.getHeading(), previousHeading);
        previousHeading = mt2poseGood.getHeading();
        calculatedPose = mt2poseGood;
        odo.setPosition(PoseConverter.poseToPose2D(mt2poseGood, PedroCoordinates.INSTANCE));
        currentVelocity = new Pose(odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH), odo.getHeading(AngleUnit.RADIANS)); // TODO: Maybe calculate velocity with megatag
    }

    public void updateOnPinpoint(){
        odo.update();
        Pose currentPinpointPose = PoseConverter.pose2DToPose(odo.getPosition(), PedroCoordinates.INSTANCE);
        totalHeading += MathFunctions.getSmallestAngleDifference(currentPinpointPose.getHeading(), previousHeading);
        previousHeading = currentPinpointPose.getHeading();
        currentVelocity = new Pose(odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH), odo.getHeading(AngleUnit.RADIANS));
        calculatedPose = currentPinpointPose;
    }

    /**
     * This returns how far the robot has turned in radians, in a number not clamped between 0 and
     * 2 * pi radians. This is used for some tuning things and nothing actually within the following.
     *
     * @return returns how far the robot has turned in total, in radians.
     */
    @Override
    public double getTotalHeading() {
        return totalHeading;
    }

    /**
     * This returns the multiplier applied to forward movement measurement to convert from encoder
     * ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the forward ticks to inches multiplier
     */
    @Override
    public double getForwardMultiplier() { return odo.getEncoderY(); }

    /**
     * This returns the multiplier applied to lateral/strafe movement measurement to convert from
     * encoder ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the lateral/strafe ticks to inches multiplier
     */
    @Override
    public double getLateralMultiplier() { return odo.getEncoderX(); }

    /**
     * This returns the multiplier applied to turning movement measurement to convert from encoder
     * ticks to radians. This is found empirically through a tuner.
     *
     * @return returns the turning ticks to radians multiplier
     */
    @Override
    public double getTurningMultiplier() { return odo.getYawScalar(); }

    /**
     * This resets the IMU of the localizer, if applicable.
     */
    @Override
    public void resetIMU() {
        resetPinpoint();
    }

    private void resetPinpoint() {
        odo.resetPosAndIMU();

        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * This is overridden to return the IMU's heading estimate, if there is one.
     *
     * @return returns the IMU's heading estimate if it exists
     */
    @Override
    public double getIMUHeading() {
        return Double.NaN;
    }

    /**
     * This returns whether if any component of robot's position is NaN.
     *
     * @return returns if any component of the robot's position is NaN
     */
    @Override
    public boolean isNAN() { return Double.isNaN(getPose().getX()) || Double.isNaN(getPose().getY()) || Double.isNaN(getPose().getHeading()); }

    public void setStartPose(Pose setStart) {
        if (!Objects.equals(startPose, new Pose()) && startPose != null) {
            Pose currentPose = calculatedPose.rotate(-startPose.getHeading(), false).minus(startPose);
            setPose(setStart.plus(currentPose.rotate(setStart.getHeading(), false)));
        } else {
            setPose(setStart);
        }

        this.startPose = setStart;
    }

    //Needed to process MT2 poses
    public Pose poseFromPose3d(Pose3D pose){
        return new Pose(
                pose.getPosition().x,
                pose.getPosition().y,
                pose.getOrientation().getYaw()
        );
    }

    public GoBildaPinpointDriver getPinpoint() {
        return odo;
    }
    public Limelight3A getLimelight() {
        return ll;
    }
    public void recalibrate() {
        odo.recalibrateIMU();
    }

}
