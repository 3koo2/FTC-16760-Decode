package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OpmodeConstants;

public class SimpleIntakeSubsystem {
    DcMotorEx intakeMotor;
    Telemetry telemetry;
    int intakePower = 0;

    public SimpleIntakeSubsystem(HardwareMap hwmap, Telemetry tele){
        intakeMotor = hwmap.get(DcMotorEx.class,  OpmodeConstants.MOTOR_NAME_INTAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry = tele;
    }

    public void toggleOnOff(){
        if (intakePower == 1 || intakePower == -1) {
            setIntake(0);
            return;
        }
        setIntake(1);
    }

    public void toggleForwardBack(){
        setIntake(-intakePower);
    }

    public void setIntake(int power){
        if (power != 1 && power != 0 && power != -1) return;
        intakePower = power;
    }

    public void spinIntake(){
        intakeMotor.setPower(intakePower);
    }
}
