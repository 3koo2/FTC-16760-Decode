package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@TeleOp(name = "Fake Turret")
public class LimelightTurretAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        TurretSubsystem turret = new TurretSubsystem(hardwareMap, telemetry, null);
        double heading = 0;

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            heading += gamepad1.right_stick_x;
            turret.pointTowardsFieldCentric(
                        new Pose2D(DistanceUnit.INCH, 26,26, AngleUnit.DEGREES, 0),
                        new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, heading)
                    );
            turret.goToSetpoint();
            telemetry.update();
        }
    }
}
