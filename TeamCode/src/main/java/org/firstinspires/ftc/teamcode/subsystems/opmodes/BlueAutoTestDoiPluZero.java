package org.firstinspires.ftc.teamcode.subsystems.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pathing.WayPoint;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;

import java.util.List;

@Autonomous
public class BlueAutoTestDoiPluZero extends LinearOpMode {
    MecanumDrivetrain drive=new MecanumDrivetrain();

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap, telemetry);
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadEx detectorGamepad=new GamepadEx(gamepad1);
        FtcDashboard dashboard= FtcDashboard.getInstance();
        waitForStart();
        drive.setPositionEstimate(new Pose2d(11.83, 62.16, Rotation2d.fromDegrees(270.00)));
        while (opModeIsActive()){
            hubs.forEach(LynxModule::clearBulkCache);
            drive.updateLocalizer();
            if (detectorGamepad.wasJustPressed(GamepadKeys.Button.A)){
                drive.setTarget(new WayPoint(new Pose2d(25,42,Rotation2d.fromDegrees(270)), 1));
            }
            if (detectorGamepad.wasJustPressed(GamepadKeys.Button.B)){
                drive.setTarget(new WayPoint(new Pose2d(48, 42, Rotation2d.fromDegrees(180)), 1));
            }
            if (detectorGamepad.wasJustPressed(GamepadKeys.Button.X)){
                drive.setTarget(new WayPoint(new Pose2d(30, 12, Rotation2d.fromDegrees(180)), 1));
            }
            if (detectorGamepad.wasJustPressed(GamepadKeys.Button.Y)){
                drive.setTarget(new WayPoint(new Pose2d(-12.5, 12, Rotation2d.fromDegrees(180)), 1));
            }
            drive.updatePIDS();
            telemetry.addData("At target", drive.atTarget());
            telemetry.addData("heading", drive.localizer.getPose().getHeading());
            telemetry.update();
            detectorGamepad.readButtons();
            TelemetryPacket packet = new TelemetryPacket();
            Pose2d position=drive.localizer.getPose();

            packet.fieldOverlay().setFill("blue")
                    .strokeCircle(position.getX(), position.getY(), 9)
                    .strokeLine(position.getX(), position.getY(),
                            (position.getRotation().getCos()*10)+ position.getX(),
                            (position.getRotation().getSin()*10)+ position.getY());

            dashboard.sendTelemetryPacket(packet);
        }
    }
}
