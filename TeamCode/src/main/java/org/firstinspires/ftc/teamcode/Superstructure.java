package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MinimalIntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MinimalLauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class Superstructure{
    public TurretSubsystem turret;
    public IntakeSubsystem intake;
    public MinimalIntakeSubsystem mintake;

    public DriveSubsystem drive;
    public LauncherSubsystem launcher;
    public MinimalLauncherSubsystem mlauncher;

    public LimelightSubsystem limelightSubsystem;
    public Follower pedro;

    public GoBildaPinpointDriver pinpoint;

    private Telemetry telemetry;
    public Superstructure(HardwareMap hwmap, Telemetry tele){
        this.telemetry = tele;

        // temporary disable
        // this.turret = new TurretSubsystem(hwmap, tele);
        this.mintake = new MinimalIntakeSubsystem(hwmap, tele);
        this.drive = new DriveSubsystem(hwmap, tele);
        this.mlauncher = new MinimalLauncherSubsystem(hwmap, tele);
        //this.pinpoint = hwmap.get(GoBildaPinpointDriver.class, OpmodeConstants.PINPOINT_NAME);
        //this.limelightSubsystem = new LimelightSubsystem(hwmap, tele, null);
        //this.pedro = Constants.createFollower(hwmap);
    }
}