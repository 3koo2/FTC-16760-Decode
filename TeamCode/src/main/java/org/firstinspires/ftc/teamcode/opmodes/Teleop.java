package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Superstructure;

@TeleOp(name="16760 Teleop")
public class Teleop extends LinearOpMode {
    private Superstructure superstructure;
    //private IMU imu;

    //private Limelight3A ll;

    public void runOpMode(){
        this.superstructure = new Superstructure(hardwareMap, telemetry);
        //this.imu = hardwareMap.get(IMU.class, "imu");
        //this.ll = hardwareMap.get(Limelight3A.class, "limelight");

        //imu.resetYaw();
        //ll.pipelineSwitch(0);
        telemetry.addLine("Initializing Teleop");
        telemetry.update();

        waitForStart();

        //ll.start();
        while (opModeIsActive()){
            // ll template
            //YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            ///this.ll.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

            //LLResult res = this.ll.getLatestResult();
            //if (res!=null){
              /*  if (res.isValid()) {
                    Pose3D botpose = res.getBotpose_MT2();
                    telemetry.addData("posex", botpose.getPosition().x);
                    telemetry.addData("posey", botpose.getPosition().y);
                    telemetry.addData("posez", botpose.getPosition().z);

                }
            }*/



            this.superstructure.drive.teleopDrive(gamepad1);

            // other player controls:
            this.superstructure.mintake.operateIntake(gamepad1);
            //this.superstructure.turret.operateTurret(gamepad2);
//
//            if (gamepad1.xWasPressed()){
//                this.superstructure.mlauncher.flywheel.setVelocity(1400);
//            } else {
//                this.superstructure.mlauncher.flywheel.setVelocity(0);
//            }

            this.superstructure.mlauncher.controlLauncher(gamepad1);
            this.superstructure.mfeeder.runFeed(gamepad1);

            telemetry.update();
        }
    }
}
