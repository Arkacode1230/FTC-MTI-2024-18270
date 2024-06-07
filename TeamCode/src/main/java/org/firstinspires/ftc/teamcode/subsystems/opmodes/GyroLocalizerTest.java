package org.firstinspires.ftc.teamcode.subsystems.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.GyroLocalizer;

import java.util.List;

@TeleOp
public class GyroLocalizerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GyroLocalizer localizer=new GyroLocalizer(hardwareMap);
        FtcDashboard dashboard= FtcDashboard.getInstance();
        waitForStart();
        double loopTime=0;
        while (opModeIsActive()){
            hubs.forEach(LynxModule::clearBulkCache);
            localizer.updatePose();
            if (gamepad1.a){
                localizer.updatePose(new Pose2d());
            }
            if (gamepad1.b){
                localizer.updatePose(new Pose2d(0, 0, Rotation2d.fromDegrees(90)));
            }
            Pose2d position=localizer.getPose();
            telemetry.addLine(position.toString());
            TelemetryPacket packet = new TelemetryPacket();


            packet.fieldOverlay().setFill("blue")
                    .strokeCircle(position.getX(), position.getY(), 9)
                    .strokeLine(position.getX(), position.getY(),
                            (position.getRotation().getCos()*10)+ position.getX(),
                            (position.getRotation().getSin()*10)+ position.getY());

            dashboard.sendTelemetryPacket(packet);
            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();
        }
    }
}
