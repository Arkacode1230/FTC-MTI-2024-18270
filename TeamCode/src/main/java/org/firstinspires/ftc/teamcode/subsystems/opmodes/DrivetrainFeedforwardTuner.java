package org.firstinspires.ftc.teamcode.subsystems.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;

import java.util.List;

@TeleOp
@Config
public class DrivetrainFeedforwardTuner extends OpMode {
    MecanumDrivetrain drive=new MecanumDrivetrain();
    public static double dx, dy, dh=0;
    List<LynxModule> hubs;
    FtcDashboard dashboard;
    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        drive.init(hardwareMap, telemetry);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dashboard= FtcDashboard.getInstance();
    }
    @Override
    public void loop(){
        hubs.forEach(LynxModule::clearBulkCache);
        drive.setWeightedPowers(dx, dy, dh);
        List<Double> velocity1=drive.getVelocity();
        telemetry.addData("Measured Velocity", velocity1);
        drive.updateLocalizer();
        TelemetryPacket packet = new TelemetryPacket();
        Pose2d position=drive.localizer.getPose();
        telemetry.addData("Position", position);
        packet.fieldOverlay().setFill("blue")
                .strokeCircle(position.getX(), position.getY(), 9)
                .strokeLine(position.getX(), position.getY(),
                        (position.getRotation().getCos()*10)+ position.getX(),
                        (position.getRotation().getSin()*10)+ position.getY());

        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
    }
}
