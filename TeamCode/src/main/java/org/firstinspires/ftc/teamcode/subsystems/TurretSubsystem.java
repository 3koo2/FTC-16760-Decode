package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.OpmodeConstants;
import org.firstinspires.ftc.teamcode.lib.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.constants.TurretConstants;

public class TurretSubsystem {
    private DcMotorEx turret;
    private GoBildaPinpointDriver pinpoint;
    private int turretSetpoint = 0;

    private PIDController turretPID;

    private Telemetry telemetry;
    public TurretSubsystem(HardwareMap hwmap, Telemetry tele, GoBildaPinpointDriver pinpoint){
        this.telemetry = tele;

        this.turret = hwmap.get(DcMotorEx.class, OpmodeConstants.MOTOR_NAME_TURRET);
        this.turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.pinpoint = pinpoint;
        this.turretPID = new PIDController(TurretConstants.Kp, TurretConstants.Ki, TurretConstants.Kd);

        this.turretPID.setTelemetry(tele);
    }

    public void setRawPower(double power){
        this.turret.setPower(power);
    }

    public int getTurretPosition(){
        return this.turret.getCurrentPosition();
    }



    public boolean setSetpoint(int position){
        boolean range = TurretConstants.LOWER_LIMIT < position && position < TurretConstants.UPPER_LIMIT;
        if (range) this.turretSetpoint = position;
        return range;
    }

    public boolean moveOffset(double angle){
        return setSetpoint((int)(angle*TurretConstants.TICK_DEGREE_CONVERSION_FACTOR));
    }

    public boolean pointTowardsFieldCentric(Pose2D target){
        return pointTowardsFieldCentric(target, pinpoint.getPosition());
    }
    public boolean pointTowardsFieldCentric(Pose2D target, Pose2D robot){
        double rx = robot.getX(DistanceUnit.INCH);
        double ry = robot.getY(DistanceUnit.INCH);

        double tx = target.getX(DistanceUnit.INCH);
        double ty = target.getY(DistanceUnit.INCH);
        // x and y might be switched ngl idk sometimes they do that.
        double dx = tx-rx;
        double dy=ty-ry;

        double angle = 180*Math.atan(dx/dy)/Math.PI;
        angle += robot.getHeading(AngleUnit.DEGREES); // heading from forward-(x)
        telemetry.addData("targetangle", angle);
        return moveOffset(angle);
    }

    public void goToSetpoint(){
        // something with pid,
        // this should do something.

        int currentPosition = getTurretPosition();
        double output = turretPID.calculatePID(currentPosition, this.turretSetpoint);

        telemetry.addData("Turret PID", output);
        this.turret.setPower(output);
    }

    public void operateTurret(Gamepad gamepad2){
        double rawPower = gamepad2.left_stick_x;
        if (Math.abs(rawPower)> OpmodeConstants.TRIGGER_TOLERANCE){
            setRawPower(rawPower);
        }

        boolean sp0 = gamepad2.dpad_left;
        boolean sp1 = gamepad2.dpad_right;
        // i doubt i'll keep many of these. probably one to reset to normal 0 position, and the others are
        // auto-aligned towards the basket.
        if (sp0){
            setSetpoint(TurretConstants.SETPOINT_0);
        }
        if (sp1){
            // in the future: autotarget left depot
            setSetpoint(TurretConstants.SETPOINT_1);
        }


        // do the actual things that happen over time; event-type things.?
        // actions

        goToSetpoint();
    }
}
