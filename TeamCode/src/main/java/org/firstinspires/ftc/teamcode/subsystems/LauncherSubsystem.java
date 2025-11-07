package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OpmodeConstants;
import org.firstinspires.ftc.teamcode.lib.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.constants.LauncherConstants;

public class LauncherSubsystem {
    private Telemetry telemetry;
    private DcMotorEx flywheel;
    private DcMotorEx aim;
    private int aimSetpoint = 0;

    private double flywheelVelocity = 0;

    private PIDController aimPID, flywheelPID;

    public LauncherSubsystem(HardwareMap hwmap, Telemetry t){
        this.telemetry = t;

        this.flywheel = hwmap.get(DcMotorEx.class, OpmodeConstants.MOTOR_NAME_FLYWHEEL);
        this.aim = hwmap.get(DcMotorEx.class, OpmodeConstants.MOTOR_NAME_AIM);

        this.flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.aim.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.aimPID = new PIDController(
                LauncherConstants.aimKp,
                LauncherConstants.aimKi,
                LauncherConstants.aimKd
        );

        this.flywheelPID = new PIDController(
                LauncherConstants.flywheelKp,
                LauncherConstants.flywheelKi,
                LauncherConstants.flywheelKd
        );
    }

    public void aimLauncher(int setpoint){
        // use PID
        this.aimSetpoint = 0;
    }

    public void goToAim(){
        //use pid controller

        int position = this.aim.getCurrentPosition();
        double output = this.aimPID.calculatePID(position, this.aimSetpoint);
        this.aim.setPower(output);
        telemetry.addData("aimPID: ", output);
    }

    public void enableFlywheel(double velocity){
        this.flywheelVelocity = velocity;
    }

    public void stopFlywheel(){
        this.flywheelVelocity = 0;
    }

    public void moveFlywheel(){
        // do something with motor and PID maybe?
        double velocity = flywheel.getVelocity();

        double output = flywheelPID.calculatePID(velocity, this.flywheelVelocity);
        this.flywheel.setPower(output);
        telemetry.addData("flywheelPID", output);
    }

    public void controlLauncher(Gamepad gamepad2){

        //controls
        boolean sp0 = gamepad2.dpad_down;
        boolean sp1 = gamepad2.dpad_up;

        double launch = gamepad2.right_trigger;

        if (sp0){
            aimLauncher(LauncherConstants.SETPOINT_0);
        }
        if (sp1){
            aimLauncher(LauncherConstants.SETPOINT_1);
        }
        int speed = 10;
        double factor = gamepad2.right_stick_y;

        aimLauncher(this.aimSetpoint - (int)(factor*speed));

        // gavriel's idea:

        enableFlywheel(launch*LauncherConstants.MAXIMUM_FLYWHEEL_VELOCITY);

        // do stuff:
        moveFlywheel();
        goToAim();
    }
}
