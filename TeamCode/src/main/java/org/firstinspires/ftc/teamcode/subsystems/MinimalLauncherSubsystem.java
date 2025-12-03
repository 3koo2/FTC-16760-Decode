package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OpmodeConstants;
import org.firstinspires.ftc.teamcode.lib.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.constants.LauncherConstants;

public class MinimalLauncherSubsystem {
    private Telemetry telemetry;
    private DcMotorEx flywheel;

    private double flywheelVelocity = 0;

    private PIDController flywheelPID;

    public MinimalLauncherSubsystem(HardwareMap hwmap, Telemetry t){
        this.telemetry = t;

        this.flywheel = hwmap.get(DcMotorEx.class, OpmodeConstants.MOTOR_NAME_FLYWHEEL);

        this.flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.flywheelPID = new PIDController(
                LauncherConstants.flywheelKp,
                LauncherConstants.flywheelKi,
                LauncherConstants.flywheelKd
        ); // yo idk bout this
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

        // some call to get distance from tag:

        // run some calculation for flywheel veloci.

        double calculated_velocity = LauncherConstants.MAXIMUM_FLYWHEEL_VELOCITY;

        boolean fixedlaunch = gamepad2.right_trigger > OpmodeConstants.TRIGGER_TOLERANCE;

        if (fixedlaunch){
            enableFlywheel(calculated_velocity);
        }
        else{
            stopFlywheel();
        }

        // do stuff:
        moveFlywheel();
    }
}
