package org.firstinspires.ftc.teamcode.subsystems.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;

import java.util.List;

@TeleOp
@Config
public class LocalizingTest extends LinearOpMode {
    MecanumDrivetrain drive=new MecanumDrivetrain();

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        FtcDashboard dashboard= FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive.init(hardwareMap, telemetry, dashboard);
        drive.setPositionEstimate(new Pose2d(-36.11, 62.16, Rotation2d.fromDegrees(270.00)));
        waitForStart();
        double loopTime=0;
        while (opModeIsActive()){
            hubs.forEach(LynxModule::clearBulkCache);
            drive.updateLocalizer();
            double yaw=drive.localizer.getPose().getRotation().getRadians();
            telemetry.addData("Yaw", yaw);
            drive.setWeightedPowers(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            Pose2d position=drive.localizer.getPose();
            telemetry.addLine(position.toString());
            telemetry.addLine(drive.getVelocity().toString());
            TelemetryPacket packet = new TelemetryPacket();


            packet.fieldOverlay().setFill("blue")
                    .strokeCircle(position.getX(), position.getY(), 9)
                    .strokeLine(position.getX(), position.getY(),
                            (position.getRotation().getCos()*10)+ position.getX(),
                            (position.getRotation().getSin()*10)+ position.getY())
                    .strokeLine(position.getX(), position.getY(),
                            Math.cos(yaw)*10+ position.getX(),
                            Math.sin(yaw)*10+ position.getY());

            dashboard.sendTelemetryPacket(packet);
            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();
        }
    }
}
