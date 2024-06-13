package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.notshown.PropPosition;
import org.firstinspires.ftc.teamcode.pathing.WayPoint;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import java.util.List;

@Autonomous
@Config
public class BlueAutoFar2plus5 extends LinearOpMode {
    MecanumDrivetrain drive=new MecanumDrivetrain();
    Intake intake=new Intake();
    Outtake outtake=new Outtake();
    public static PropPosition randomization=PropPosition.RIGHT;
    @Override
    public void runOpMode() throws InterruptedException {
        intake.init(hardwareMap);
        outtake.init(hardwareMap);
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard = FtcDashboard.getInstance();
        drive.init(hardwareMap, telemetry, dashboard);
        waitForStart();
        drive.setPositionEstimate(new Pose2d(-36.11, 62.16, Rotation2d.fromDegrees(270.00)));
        WayPoint rightPurpleWaypoint = new WayPoint(new Pose2d(-48, 40, Rotation2d.fromDegrees(270)), 1);
        WayPoint leftYellowWaypoint = new WayPoint(new Pose2d(52, 36, Rotation2d.fromDegrees(180)), 1);
        WayPoint middlePurpleWaypoint = new WayPoint(new Pose2d(-40, 38, Rotation2d.fromDegrees(270)), 1);
        WayPoint middleYellowWaypoint = new WayPoint(new Pose2d(52, 28.5, Rotation2d.fromDegrees(180)), 1);
        WayPoint whiteStack = new WayPoint(new Pose2d(-40, 44, Rotation2d.fromDegrees(200)), 1);
        WayPoint whiteIntake = new WayPoint(new Pose2d(-46, 39, Rotation2d.fromDegrees(200)), 1);
        WayPoint leftPurpleWaypoint = new WayPoint(new Pose2d(-41, 33, Rotation2d.fromDegrees(-33)), 1);
        WayPoint rightYellowWaypoint = new WayPoint(new Pose2d(52, 23, Rotation2d.fromDegrees(180)), 1);
        WayPoint backdropTruss = new WayPoint(new Pose2d(-30, 51, Rotation2d.fromDegrees(180)), 1);
        WayPoint backdropTrussDone = new WayPoint(new Pose2d(35, 51, Rotation2d.fromDegrees(180)), 1);
        WayPoint whiteIntakeShake = new WayPoint(new Pose2d(-46, 36.5, Rotation2d.fromDegrees(200)), 1);
        WayPoint whiteIntakeShakeSec = new WayPoint(new Pose2d(-46.5, 35, Rotation2d.fromDegrees(205)), 1);


        intake.transferPosition();
        intake.setTarget(50);
        outtake.setPixelLatch(false);
        if (randomization==PropPosition.RIGHT) {
            drive.setTarget(rightPurpleWaypoint);

            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
            }
            drive.setTarget(whiteStack);
            intake.stay(0);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
            }
            intake.intakePosition4th();
            intake.setPower(1);
            drive.setTarget(whiteIntake);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
            }
            waitms(500);
            intake.transferPosition();
            drive.setTarget(backdropTruss);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
            }
            intake.setPower(-1);
            drive.setTarget(backdropTrussDone);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
            }
            intake.setPower(0);
            intake.stay(0);
            waitms(200);
            outtake.setPixelLatch(true);
            waitms(200);
            outtake.depositPosition(100);
            drive.setTarget(rightYellowWaypoint);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                outtake.update();
            }
            waitms(1000);
            outtake.setPixelLatch(false);
            waitms(500);
            outtake.transferPosition();
        }
        if (randomization==PropPosition.MIDDLE) {
            drive.setTarget(middlePurpleWaypoint);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
            }
            drive.setTarget(whiteStack);
            intake.stay(0);
            //outtake.depositPosition(0);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
            }
            waitms(20500);
            outtake.setPixelLatch(false);
            waitms(1000);
            outtake.transferPosition();
            drive.setTarget(new WayPoint(new Pose2d(35, 6, Rotation2d.fromDegrees(180)), 2));
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
            }
        }
        if (randomization==PropPosition.LEFT) {
            drive.setTarget(leftPurpleWaypoint);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
            }
            drive.setTarget(whiteStack);
            intake.stay(0);
            //outtake.depositPosition(0);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
            }
        }
        drive.setTarget(backdropTrussDone);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
            outtake.update();
        }
        drive.setTarget(backdropTruss);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
        }
        drive.setTarget(whiteStack);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
        }
        intake.intakePosition2nd(0);
        intake.setPower(1);
        drive.setTarget(whiteIntake);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
        }
        waitms(200);
        drive.setTarget(whiteIntakeShake);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
        }
        waitms(300);
        intake.transferPosition();
        drive.setTarget(backdropTruss);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
        }
        intake.setPower(-1);
        drive.setTarget(backdropTrussDone);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
        }
        intake.setPower(0);
        intake.stay(0);
        waitms(200);
        outtake.setPixelLatch(true);
        waitms(200);
        outtake.depositPosition(100);
        drive.setTarget(middleYellowWaypoint);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            outtake.update();
        }
        waitms(200);
        outtake.setPixelLatch(false);
        waitms(500);
        outtake.transferPosition();
        drive.setTarget(backdropTrussDone);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
            outtake.update();
        }
        drive.setTarget(backdropTruss);
        intake.intakePosition(0);
        intake.setPower(1);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
        }
        drive.setTarget(whiteStack);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
        }
        drive.setTarget(whiteIntake);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
        }
        waitms(200);
        drive.setTarget(whiteIntakeShakeSec);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
        }
        waitms(300);
        intake.transferPosition();
        drive.setTarget(backdropTruss);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
        }
        intake.setPower(-1);
        drive.setTarget(backdropTrussDone);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
        }
        intake.setPower(0);
        intake.stay(0);
        waitms(200);
        outtake.setPixelLatch(true);
        waitms(200);
        outtake.depositPosition(100);
        drive.setTarget(middleYellowWaypoint);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            outtake.update();
        }
        waitms(3000);
        outtake.setPixelLatch(false);
        waitms(500);
    }
    public void waitms(long ms){
        ElapsedTime timer=new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds()<ms){
            intake.update();
            outtake.update();
        }
        
    }
}
