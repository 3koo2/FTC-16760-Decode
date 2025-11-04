package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.constants.IntakeConstants;

public class IntakeSubsystem {
    private DcMotor intakeMotor;
    private Servo door0;
    private Servo door1;
    private Servo door2;

    private boolean toggle_intakeOn;
    private boolean intaking;

    public enum Door{
        DOOR0,
        DOOR1,
        DOOR2
    };

    public IntakeSubsystem(HardwareMap hwmap, Telemetry tele){
        this.intakeMotor = hwmap.get(DcMotor.class, Constants.MOTOR_NAME_INTAKE);
        this.door0 = hwmap.get(Servo.class, Constants.SERVO_NAME_DOOR0);
        this.door1 = hwmap.get(Servo.class, Constants.SERVO_NAME_DOOR1);
        this.door2 = hwmap.get(Servo.class, Constants.SERVO_NAME_DOOR2);

        this.intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.toggle_intakeOn = false;
        this.intaking = false;
    }

    public void closeAllDoors(){
        this.door0.setPosition(IntakeConstants.DOOR_0_CLOSED);
        this.door1.setPosition(IntakeConstants.DOOR_1_CLOSED);
        this.door2.setPosition(IntakeConstants.DOOR_2_CLOSED);
    }

    public void openDoor(Door door){
       switch (door){
           case DOOR0:
               this.door0.setPosition(IntakeConstants.DOOR_0_OPEN);
               break;
           case DOOR1:
               this.door1.setPosition(IntakeConstants.DOOR_1_OPEN);
               break;
           case DOOR2:
               this.door2.setPosition(IntakeConstants.DOOR_2_OPEN);
               break;
       }
    }

    public void closeDoor(Door door){
        switch (door){
            case DOOR0:
                this.door0.setPosition(IntakeConstants.DOOR_0_CLOSED);
                break;
            case DOOR1:
                this.door1.setPosition(IntakeConstants.DOOR_1_CLOSED);
                break;
            case DOOR2:
                this.door2.setPosition(IntakeConstants.DOOR_2_CLOSED);
                break;
        }
    }

    public void feedIntake(){
        this.intakeMotor.setPower(1);
        // this.intakeMotor.setPower(-1);
    }

    public void stopIntake(){
        this.intakeMotor.setPower(0);
    }

    public void operateIntake(Gamepad gamepad2){
        boolean close_all_gates = gamepad2.right_bumper;//gamepad2.right_trigger > Constants.TRIGGER_TOLERANCE;
        boolean open_all_gates = gamepad2.left_bumper;//gamepad2.left_trigger > Constants.TRIGGER_TOLERANCE;

        boolean intake_toggle = gamepad2.a;

        boolean gate0 = gamepad2.x;
        boolean gate1 = gamepad2.y;
        boolean gate2 = gamepad2.b;

        if (close_all_gates){
            closeAllDoors();
        }

        if (open_all_gates){
            openDoor(Door.DOOR0);
            openDoor(Door.DOOR1);
            openDoor(Door.DOOR2);
        }

        // toggle to turn intake spinner on/off
        if (!toggle_intakeOn && intake_toggle){
            if (intaking){
                stopIntake();
                intaking = false;
            }
            else {
                feedIntake();
                intaking = true;
            }
        }
        toggle_intakeOn = intake_toggle;

        // press the remaining buttons to open the three door/ gates

        if (gate0){
            openDoor(Door.DOOR0);
        }
        if (gate1){
            openDoor(Door.DOOR1);
        }
        if (gate2){
            openDoor(Door.DOOR2);
        }
    }
}
