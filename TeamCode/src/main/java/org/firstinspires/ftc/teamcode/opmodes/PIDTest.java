package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.lib.PIDController;

@Configurable
@TeleOp(name="chOpMode")
public class PIDTest extends LinearOpMode {
    public static double kp = 0.006;
    public static double ki = 0;
    public static double kd = 0;

    public static int ticksperrotation = 538;
    public static int howmanyrotations = 10;



    public void runOpMode(){
        PIDController pid = new PIDController(kp ,ki,kd);
        DcMotor motor = hardwareMap.get(DcMotor.class, "motor");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        int setpoint = ticksperrotation*howmanyrotations;
        pid.setTelemetry(telemetry);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("setpoint",setpoint);
            telemetry.addData("currentposition",motor.getCurrentPosition());


            if (!gamepad1.b) {
                double calc = pid.calculatePID(motor.getCurrentPosition(), setpoint);
                telemetry.addData("calculated",calc);
                motor.setPower(calc);
            }
            else{
                motor.setPower(0);
                pid.resetIntegral();
            }

            telemetry.addData("gamepada",gamepad1.a);

            telemetry.update();

            if (gamepad1.a){
                pid.resetIntegral();
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }
}
