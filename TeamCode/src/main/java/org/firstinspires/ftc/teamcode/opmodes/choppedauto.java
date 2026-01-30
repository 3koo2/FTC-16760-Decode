package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Superstructure;

@Autonomous(name="Leave AUto")
public class choppedauto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Superstructure ss = new Superstructure(hardwareMap, telemetry);
        waitForStart();
        ss.drive.set(0.5,0.5,0.5,0.5);
        sleep(2000);
    }
}
