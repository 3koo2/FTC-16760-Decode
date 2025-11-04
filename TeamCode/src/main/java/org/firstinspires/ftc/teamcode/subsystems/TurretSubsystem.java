package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.lib.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.constants.TurretConstants;

public class TurretSubsystem {
    private DcMotorEx turret;
    private int turretSetpoint = 0;

    private PIDController turretPID;

    private Telemetry telemetry;
    public TurretSubsystem(HardwareMap hwmap, Telemetry tele){
        this.telemetry = tele;

        this.turret = hwmap.get(DcMotorEx.class, Constants.MOTOR_NAME_TURRET);
        this.turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.turretPID = new PIDController(TurretConstants.Kp, TurretConstants.Ki, TurretConstants.Kd);
    }

    public void setRawPower(double power){
        this.turret.setPower(power);
    }

    public int getTurretPosition(){
        return this.turret.getCurrentPosition();
    }

    public void getTurretPositionInSpace(){
        //define later (field-centric, maybe idk)
    }

    public void setSetpoint(int position){
        this.turretSetpoint = position;
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
        if (Math.abs(rawPower)>Constants.TRIGGER_TOLERANCE){
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
