package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.OpmodeConstants;

public class FeedSubsystem {
    private Servo feed0;
    private Servo feed1;

    private Telemetry telemetry;
    public FeedSubsystem(HardwareMap hwmap, Telemetry telemetry){
        feed0 = hwmap.get(Servo.class, OpmodeConstants.SERVO_NAME_FEED0);
        feed1 = hwmap.get(Servo.class, OpmodeConstants.SERVO_NAME_FEED1);

        // two continuous servos
        feed0.setDirection(Servo.Direction.FORWARD);
        feed1.setDirection(Servo.Direction.FORWARD);


    }

    public void run(){
        feed0.setPosition(1);
        feed1.setPosition(1);
    }

    public void stop(){
        feed0.setPosition(0.5);
        feed1.setPosition(0.5);
    }

    public void runFeed(Gamepad gamepad){
        if (true){} ///todo: this
    }
}
