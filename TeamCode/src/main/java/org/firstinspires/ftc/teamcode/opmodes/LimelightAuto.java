package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.lib.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

@Autonomous(name="LimelightAuto")
public class LimelightAuto extends LinearOpMode {
    private LimelightSubsystem limelight;
    private DriveSubsystem driveSubsystem;


    public void runOpMode(){
        double lastTx=0;
        double lastT2x=0;

        PIDController pid = new PIDController(0.1,0,0);

        this.limelight = new LimelightSubsystem(hardwareMap, telemetry, null);
        this.driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()){
            LLResultTypes.FiducialResult result = limelight.getTagId();


            if (result != null) {
                double tx = result.getTargetXDegrees();
                double ty = result.getTargetYDegrees();






                if (tx > 10) {
                    double m = 1;
                    driveSubsystem.set(0.5*m,-0.5*m,0.5*m,-0.5*m);
                    telemetry.addData("pid",m);
                } else if (tx < -10) {
                    double m = -1;
                    telemetry.addData("pid",m);
                    driveSubsystem.set(0.5*m,-0.5*m,0.5*m,-0.5*m);
                }
                else{
                    driveSubsystem.set(0,0,0,0);
                }

                telemetry.addData("id", result.getFiducialId());
                telemetry.addData("tx", tx);
                telemetry.addData("ty", ty);

                lastT2x = lastTx;
                lastTx = tx;
            } else{
                double m = 1;
                if (lastTx - lastT2x < 0){
                    m = -1;
                }
                driveSubsystem.set(0.5*m,-0.5*m,0.5*m,-0.5*m);
                telemetry.addLine("Chopped");
            }

            telemetry.update();
        }
    }
}
