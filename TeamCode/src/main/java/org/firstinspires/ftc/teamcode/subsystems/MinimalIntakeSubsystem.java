package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OpmodeConstants;
import org.firstinspires.ftc.teamcode.subsystems.constants.IntakeConstants;

public class MinimalIntakeSubsystem{
    private DcMotor intakeMotor;
    private Servo feed0;
    private Servo feed1; // cr
    private Servo feed2;

    private boolean toggle_intakeOn;
    private boolean intaking;

    public MinimalIntakeSubsystem(HardwareMap hwmap, Telemetry tele){
        this.intakeMotor = hwmap.get(DcMotor.class, OpmodeConstants.MOTOR_NAME_INTAKE);
//        this.feed0 = hwmap.get(Servo.class, OpmodeConstants.SERVO_NAME_FEED0);
//        this.feed1 = hwmap.get(Servo.class, OpmodeConstants.SERVO_NAME_FEED1);
//        this.feed2 = hwmap.get(Servo.class, OpmodeConstants.SERVO_NAME_FEED2);

        this.intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        this.toggle_intakeOn = false;
        this.intaking = false;
    }

    public void runIntake(double mult){
//        double p0 = mult;
//        double p1 = -mult;
//        double p2 = mult;
//
//        p0+=1;
//        p1+=1;
//        p2+=1;
//
//        p0/=2;
//        p1/=2;
//        p2/=1;
//
//        feed0.setPosition(p0);
//        feed1.setPosition(p1);
//        feed2.setPosition(p2);

        this.intakeMotor.setPower(mult);
    }

    public void operateIntake(Gamepad gamepad2){
        boolean runIntake = gamepad2.left_trigger > OpmodeConstants.TRIGGER_TOLERANCE
                || gamepad2.right_trigger >OpmodeConstants.TRIGGER_TOLERANCE;

        boolean goback = gamepad2.x; // if x is pressed, reverse intake for some reason.

        if (runIntake){
            runIntake(1);
        }
        else if (goback){
            runIntake(-1);
        }
        else{
            runIntake(0);
        }
    }
}
