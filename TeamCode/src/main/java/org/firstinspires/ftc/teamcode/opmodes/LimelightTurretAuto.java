package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@TeleOp(name = "Fake Turret")
public class LimelightTurretAuto extends LinearOpMode {
    @Override
    public void runOpMode() {

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");


        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setOffsets(3.5,9.5,DistanceUnit.INCH);
        pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();
        TurretSubsystem turret = new TurretSubsystem(hardwareMap, telemetry, pinpoint);
        Pose2D tagPose = new Pose2D(DistanceUnit.INCH, 48,48, AngleUnit.DEGREES, 0);
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, telemetry);

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            drive.teleopDrive(gamepad1);
            turret.pointTowardsFieldCentric(tagPose);
            turret.goToSetpoint();
            telemetry.update();
        }
    }
}
