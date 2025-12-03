package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Superstructure;
@Autonomous(name = "16760 Auto")
@Disabled
public class Auto extends LinearOpMode {
    private Superstructure superstructure;

    enum AutoStates{
        READ_TAG,
        LINE_UP_FIRST_SHOTS,
        FIRST_SHOOT,
        LINE_UP_INTAKE,
        INTAKE_LINE,
        LINE_UP_SECOND_SHOTS,
        SECOND_SHOOT,
        END
    }

    AutoStates state;
    double obeliskID = 0;

    @Override
    public void runOpMode() {
        this.superstructure = new Superstructure(hardwareMap, telemetry);
        state = AutoStates.READ_TAG;

        waitForStart();

        while (opModeIsActive()){
            switch (state) {
                case READ_TAG:
                    double read = superstructure.limelightSubsystem.getTagId().getFiducialId();
                    if (read != 0){
                        obeliskID = read;
                        state = AutoStates.FIRST_SHOOT;
                    }
                case LINE_UP_FIRST_SHOTS:
                    //TODO: Line up to shoot the starting balls
                    break;
                case FIRST_SHOOT:
                    superstructure.launcher.moveFlywheel();
                    break;
                case LINE_UP_INTAKE:
                    //TODO: Drive up to the line we intake
                    break;
                case INTAKE_LINE:
                    //TODO: Intake the line
                    break;
                case LINE_UP_SECOND_SHOTS:
                    //TODO: Line up the second set of shots
                    break;
                case SECOND_SHOOT:
                    //TODO: Shoot for a second time
                    break;
            }
            if (state == AutoStates.END){
                break;
            }
        }
    }
}
