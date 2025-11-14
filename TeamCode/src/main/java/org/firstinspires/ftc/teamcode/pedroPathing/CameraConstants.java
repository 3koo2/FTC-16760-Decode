package org.firstinspires.ftc.teamcode.pedroPathing;

import android.annotation.SuppressLint;

import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.OptionalDouble;

public class CameraConstants {
    public  double forwardPodY = 1;

    /** The X Offset of the Strafe Encoder (Deadwheel) from the center of the robot in DistanceUnit
     * @see #distanceUnit
     * Default Value: -2.5 */
    public  double strafePodX = -2.5;

    /** The Unit of Distance that the Pinpoint uses to measure distance
     * Default Value: DistanceUnit.INCH */
    public DistanceUnit distanceUnit = DistanceUnit.INCH;

    /** The name of the Pinpoint in the hardware map (name of the I2C port it is plugged into)
     * Default Value: "pinpoint" */
    public  String hardwareMapName = "pinpoint";
    public String limelightName = "limelight";
    public int limelightPipeline = 0;
    public double maxLimelightAngleVelocity = 10;
    public long maxLimelightUpdateTime = 10;

    /** Custom Yaw Scalar for the Pinpoint (overrides the calibration of the Pinpoint) */
    @SuppressLint("NewApi")
    public OptionalDouble yawScalar = OptionalDouble.empty();

    /** The Encoder Resolution for the Pinpoint. Used by default, but can be changed to a custom resolution.
     * Default Value: GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD */
    public  GoBildaPinpointDriver.GoBildaOdometryPods encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    /** The Encoder Resolution for the Pinpoint. Unused by default, but can be used if you want to use a custom encoder resolution. */
    @SuppressLint("NewApi")
    public OptionalDouble customEncoderResolution = OptionalDouble.empty();

    /** The Encoder Direction for the Forward Encoder (Deadwheel)
     * Default Value: GoBildaPinpointDriver.EncoderDirection.REVERSED */
    public  GoBildaPinpointDriver.EncoderDirection forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;

    /** The Encoder Direction for the Strafe Encoder (Deadwheel)
     * Default Value: GoBildaPinpointDriver.EncoderDirection.FORWARD */
    public  GoBildaPinpointDriver.EncoderDirection strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

    /**
     * This creates a new PinpointConstants with default values.
     */
    public CameraConstants() {
        defaults();
    }

    public CameraConstants forwardPodY(double forwardPodY) {
        this.forwardPodY = forwardPodY;
        return this;
    }

    public CameraConstants strafePodX(double strafePodX) {
        this.strafePodX = strafePodX;
        return this;
    }

    public CameraConstants distanceUnit(DistanceUnit distanceUnit) {
        this.distanceUnit = distanceUnit;
        return this;
    }

    public CameraConstants hardwareMapName(String hardwareMapName) {
        this.hardwareMapName = hardwareMapName;
        return this;
    }

    public CameraConstants yawScalar(double yawScalar) {
        this.yawScalar = OptionalDouble.of(yawScalar);
        return this;
    }

    public CameraConstants encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods encoderResolution) {
        this.encoderResolution = encoderResolution;
        return this;
    }

    public CameraConstants customEncoderResolution(double customEncoderResolution) {
        this.customEncoderResolution = OptionalDouble.of(customEncoderResolution);
        return this;
    }

    public CameraConstants forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection forwardEncoderDirection) {
        this.forwardEncoderDirection = forwardEncoderDirection;
        return this;
    }

    public CameraConstants strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection strafeEncoderDirection) {
        this.strafeEncoderDirection = strafeEncoderDirection;
        return this;
    }

    public CameraConstants limelightName(String limelightName){
        this.limelightName = limelightName;
        return this;
    }

    public CameraConstants limelightPipeline(int limelightPipeline){
        this.limelightPipeline = limelightPipeline;
        return this;
    }

    public CameraConstants maxLimelightAngleVelocity(double maxLimelightAngleVelocity){
        this.maxLimelightAngleVelocity = maxLimelightAngleVelocity;
        return this;
    }

    public CameraConstants maxLimelightUpdateTime(long maxLimelightUpdateTime){
        this.maxLimelightUpdateTime = maxLimelightUpdateTime;
        return this;
    }

    public void defaults() {
        forwardPodY = 1;
        strafePodX = -2.5;
        distanceUnit = DistanceUnit.INCH;
        hardwareMapName = "pinpoint";
        limelightName = "limelight";
        yawScalar = OptionalDouble.empty();
        encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        customEncoderResolution = OptionalDouble.empty();
        forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }
}
