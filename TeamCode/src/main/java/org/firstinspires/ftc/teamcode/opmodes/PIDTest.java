package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.lib.PIDController;

@TeleOp(name="chOpMode")
public class PIDTest extends LinearOpMode {

    public void runOpMode(){
        PIDController pid = new PIDController(0,0,0);
        DcMotor motor = hardwareMap.get(DcMotor.class, "motor");
        int setpoint = 1000;
        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("setpoint",setpoint);
            telemetry.addData("currentposition",motor.getCurrentPosition());

            double calc = pid.calculatePID(motor.getCurrentPosition(), setpoint);
            telemetry.addData("calculated",calc);
            motor.setPower(calc);
            telemetry.update();
        }
    }
}
