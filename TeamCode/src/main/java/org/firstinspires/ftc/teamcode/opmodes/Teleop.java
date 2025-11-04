package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Superstructure;

@TeleOp(name="16760 Teleop")
public class Teleop extends LinearOpMode {
    private Superstructure superstructure;

    public void runOpMode(){
        this.superstructure = new Superstructure(hardwareMap, telemetry);

        telemetry.addLine("Initializing Teleop");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            this.superstructure.drive.teleopDrive(gamepad1);

            // other player controls:
            this.superstructure.intake.operateIntake(gamepad2);
            this.superstructure.turret.operateTurret(gamepad2);
            this.superstructure.launcher.controlLauncher(gamepad2);

            telemetry.update();
        }
    }
}
