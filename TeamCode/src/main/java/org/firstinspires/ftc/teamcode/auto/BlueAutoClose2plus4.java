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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auto.notshown.PropPosition;
import org.firstinspires.ftc.teamcode.pathing.WayPoint;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import java.util.List;

@Autonomous
@Config
public class BlueAutoClose2plus4 extends LinearOpMode {
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
        drive.setPositionEstimate(new Pose2d(11.83, 62.16, Rotation2d.fromDegrees(270.00)));
        WayPoint leftPurpleWaypoint = new WayPoint(new Pose2d(26, 42, Rotation2d.fromDegrees(270)), 1);
        WayPoint leftYellowWaypoint = new WayPoint(new Pose2d(55, 36, Rotation2d.fromDegrees(180)), 1);
        WayPoint middlePurpleWaypoint = new WayPoint(new Pose2d(12, 38, Rotation2d.fromDegrees(270)), 1);
        WayPoint middleYellowWaypoint = new WayPoint(new Pose2d(55.5, 28.5, Rotation2d.fromDegrees(180)), 1);

        WayPoint rightCenterWaypoint = new WayPoint(new Pose2d(22, 32, Rotation2d.fromDegrees(225)), 2);
        WayPoint rightPurpleWaypoint = new WayPoint(new Pose2d(20, 30, Rotation2d.fromDegrees(180)), 1);
        WayPoint rightYellowWaypoint = new WayPoint(new Pose2d(54.5, 23, Rotation2d.fromDegrees(180)), 1);

        intake.transferPosition();
        intake.setTarget(50);
        outtake.setPixelLatch(true);
        if (randomization==PropPosition.LEFT) {
            drive.setTarget(leftPurpleWaypoint);

            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
            }
            drive.setTarget(leftYellowWaypoint);
            intake.stay(0);
            outtake.depositPosition(0);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
            }
            waitms(500);
            outtake.setPixelLatch(false);
            waitms(1000);
            outtake.transferPosition();
            drive.setTarget(new WayPoint(new Pose2d(40, 6, Rotation2d.fromDegrees(180)), 2));
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
            }
        }
        if (randomization==PropPosition.MIDDLE) {
            drive.setTarget(middlePurpleWaypoint);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
            }
            drive.setTarget(middleYellowWaypoint);
            intake.stay(0);
            outtake.depositPosition(0);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
            }
            waitms(500);
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
        if (randomization==PropPosition.RIGHT) {
            drive.setTarget(rightCenterWaypoint);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
            }
            drive.setTarget(rightPurpleWaypoint);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
            }
            drive.setTarget(rightYellowWaypoint);
            intake.stay(0);
            outtake.depositPosition(0);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
            }
            waitms(750);
            outtake.setPixelLatch(false);
            waitms(1000);
            outtake.transferPosition();
            drive.setTarget(new WayPoint(new Pose2d(30, 6, Rotation2d.fromDegrees(180)), 2));
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
            }
        }

        drive.setTarget(new WayPoint(new Pose2d(-11, 3.5, Rotation2d.fromDegrees(178.6)), 0.5));//to intake from stacks
        intake.intakePosition5th(950);
        intake.setPower(1);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
        }
        intake.intakePosition4th();//change heights
        waitms(1000);
        intake.intakePositionExtended(900);
        waitms(500);
        intake.setTarget(950);//back and forth
        waitms(1000);
        drive.setTarget(new WayPoint(new Pose2d(30, 6, Rotation2d.fromDegrees(180)), 2));//come back from stack intaking
        intake.stay(0);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
        }
        intake.transferPosition();
        waitms(500);
        intake.setPower(-1);//transfer sequence
        waitms(900);
        intake.setPower(0);
        drive.setTarget(new WayPoint(new Pose2d(51, 26, Rotation2d.fromDegrees(180)), 1));//to backdrop first time after intaking
        intake.stay(0);
        outtake.setPixelLatch(true);
        waitms(300);
        outtake.depositPosition(300);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            outtake.update();
            intake.update();
        }
        waitms(1000);
        outtake.setPixelLatch(false);//release 1st cycle
        waitms(1000);
        outtake.transferPosition();
        if (randomization==PropPosition.LEFT) {//position of intermediate point changes, so we need these if statements
            drive.setTarget(new WayPoint(new Pose2d(40, 6, Rotation2d.fromDegrees(180)), 2));
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
            }
        }
        if (randomization==PropPosition.MIDDLE) {
            drive.setTarget(new WayPoint(new Pose2d(35, 6, Rotation2d.fromDegrees(180)), 2));
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
            }
        }
        if (randomization==PropPosition.RIGHT) {
            drive.setTarget(new WayPoint(new Pose2d(30, 6, Rotation2d.fromDegrees(180)), 2));
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
            }
        }

        drive.setTarget(new WayPoint(new Pose2d(-12.5, 3.5, Rotation2d.fromDegrees(178.6)), 0.5));//intake from stack second time
        intake.intakePositionExtended(970);
        intake.setPower(1);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
        }
        waitms(200);
        intake.setTarget(800);//back and forth
        waitms(400);
        intake.setTarget(970);
        waitms(900);
        drive.setTarget(new WayPoint(new Pose2d(30, 6, Rotation2d.fromDegrees(180)), 2));//going back to the backdrop 2nd time
        intake.stay(0);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
        }
        intake.transferPosition();
        waitms(500);
        intake.setPower(-1);//transfer sequence second time
        waitms(900);
        intake.setPower(0);
        drive.setTarget(new WayPoint(new Pose2d(51, 26, Rotation2d.fromDegrees(180)), 1));//to backdrop second time
        intake.stay(0);
        outtake.setPixelLatch(true);
        waitms(300);
        outtake.depositPosition(300);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            outtake.update();
            intake.update();
        }
        waitms(1000);
        outtake.setPixelLatch(false);
        waitms(1000);
        outtake.transferPosition();
        waitms(1000);
    }
    public void waitms(long ms){
        ElapsedTime timer=new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds()<ms){
            intake.update();
            outtake.update();
            drive.updateLocalizer();
            drive.updatePIDS();

        }
        
    }
}
