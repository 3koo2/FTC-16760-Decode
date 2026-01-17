package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.OpmodeConstants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Autonomous (name="chopped", group="fuckmylife")
public class choppedauto extends LinearOpMode {
    public DcMotor front_left;
    public DcMotor front_right;
    public DcMotor back_left;
    public DcMotor back_right;

    @Override
    public void runOpMode() throws InterruptedException {
        front_left = hardwareMap.get(DcMotor.class, OpmodeConstants.MOTOR_NAME_FL);
        front_right = hardwareMap.get(DcMotor.class, OpmodeConstants.MOTOR_NAME_FR);
        back_left = hardwareMap.get(DcMotor.class, OpmodeConstants.MOTOR_NAME_BL);
        back_right = hardwareMap.get(DcMotor.class, OpmodeConstants.MOTOR_NAME_BR);

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        back_left.setPower(0.5);
        back_right.setPower(0.5);
        front_left.setPower(0.5);
        front_right.setPower(0.5);
        sleep(2000);
    }
}
