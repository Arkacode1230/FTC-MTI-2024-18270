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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.notshown.BluePipeline;
import org.firstinspires.ftc.teamcode.auto.notshown.PropPosition;
import org.firstinspires.ftc.teamcode.pathing.WayPoint;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.Objects;

@Autonomous(preselectTeleOp = "Teleop")
@Config
public class BlueAutoClose2plus4Truss extends LinearOpMode {
    MecanumDrivetrain drive=new MecanumDrivetrain();
    Intake intake=new Intake();
    Outtake outtake=new Outtake();
    public static PropPosition randomization=PropPosition.RIGHT;
    OpenCvWebcam webcam;
    public static String ObjectDirection;
    @Override
    public void runOpMode() throws InterruptedException {
        intake.init(hardwareMap);
        outtake.init(hardwareMap);
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard = FtcDashboard.getInstance();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        BluePipeline pipeline = new BluePipeline(telemetry, ObjectDirection);
        webcam.setPipeline(pipeline);
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        drive.init(hardwareMap, telemetry, dashboard);

        while (opModeInInit()){
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("Outtake encoder Pos", outtake.getEncoderPos());

            telemetry.update();

            if (Objects.equals(pipeline.getPosition(), "LEFT")) {
                telemetry.addData("Position", "LEFTpo");
                randomization=PropPosition.LEFT;
            }
            else if (Objects.equals(pipeline.getPosition(), "MIDDLE")){
                telemetry.addData("Position", "MIDDLEE");
                randomization=PropPosition.MIDDLE;
            }
            else if (Objects.equals(pipeline.getPosition(), "RIGHT")){
                telemetry.addData("Position", "RIGHTO");
                randomization=PropPosition.RIGHT;
            }

            sleep(100);
        }

        webcam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener()
        {
            @Override
            public void onClose() {

            }
        });

        waitForStart();
        outtake.resetEncoder();
        intake.resetEncoder();

        drive.setPositionEstimate(new Pose2d(11.83, 62.16, Rotation2d.fromDegrees(270.00)));
        WayPoint whiteStack = new WayPoint(new Pose2d(-40, 43.5, Rotation2d.fromDegrees(210)), 3);
        WayPoint whiteIntake = new WayPoint(new Pose2d(-46, 39.5, Rotation2d.fromDegrees(210)), 1);
        WayPoint backdropTruss = new WayPoint(new Pose2d(-30, 49, Rotation2d.fromDegrees(180)), 1);
        WayPoint backdropTrussDone = new WayPoint(new Pose2d(35, 49, Rotation2d.fromDegrees(180)), 1);
        WayPoint backdropTrussSusCode = new WayPoint(new Pose2d(-17, 49, Rotation2d.fromDegrees(180)), 1);
        WayPoint whiteIntakeShake = new WayPoint(new Pose2d(-44, 37.5, Rotation2d.fromDegrees(200)), 1);
        WayPoint whiteIntakeShakeSec = new WayPoint(new Pose2d(-47, 37.5, Rotation2d.fromDegrees(210)), 1);
        WayPoint leftPurpleWaypoint = new WayPoint(new Pose2d(25, 42, Rotation2d.fromDegrees(270)), 1);
        WayPoint leftYellowWaypoint = new WayPoint(new Pose2d(56, 36, Rotation2d.fromDegrees(180)), 1);
        WayPoint middlePurpleWaypoint = new WayPoint(new Pose2d(14, 38, Rotation2d.fromDegrees(270)), 0.5);
        WayPoint middleYellowWaypoint = new WayPoint(new Pose2d(55.5, 27.5, Rotation2d.fromDegrees(180)), 1);
        WayPoint whiteIntakeShakeInOut = new WayPoint(new Pose2d(-35, 45, Rotation2d.fromDegrees(200)), 2);
        WayPoint rightCenterWaypoint = new WayPoint(new Pose2d(22, 30, Rotation2d.fromDegrees(225)), 2);
        WayPoint rightPurpleWaypoint = new WayPoint(new Pose2d(20, 27, Rotation2d.fromDegrees(180)), 1);
        WayPoint rightYellowWaypoint = new WayPoint(new Pose2d(55, 20, Rotation2d.fromDegrees(180)), 1);
        WayPoint whiteIntakeShakeSecFarther = new WayPoint(new Pose2d(-47, 35.5, Rotation2d.fromDegrees(210)), 1);

        intake.transferPosition();
        intake.setTarget(50);
        outtake.setPixelLatch(true);
        intake.setPower(0.5);
        if (randomization==PropPosition.LEFT) {
            drive.setTarget(leftPurpleWaypoint);

            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
            intake.setPower(0);
            drive.setTarget(leftYellowWaypoint);
            intake.stay(0);
            outtake.depositPosition(0);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
            waitms(500);
            outtake.setPixelLatch(false);
            waitms(1000);
            outtake.transferPosition();
        }
        if (randomization==PropPosition.MIDDLE) {
            drive.setTarget(middlePurpleWaypoint);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
            intake.setPower(0);
            drive.setTarget(middleYellowWaypoint);
            intake.stay(0);
            outtake.depositPosition(0);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
            waitms(500);
            outtake.setPixelLatch(false);
            waitms(1000);
            outtake.transferPosition();
        }
        if (randomization==PropPosition.RIGHT) {
            drive.setTarget(rightCenterWaypoint);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
            intake.setPower(0);
            drive.setTarget(rightPurpleWaypoint);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
            drive.setTarget(rightYellowWaypoint);
            intake.stay(0);
            outtake.depositPosition(0);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
            waitms(750);
            outtake.setPixelLatch(false);
            waitms(1000);
            outtake.transferPosition();
        }
        /////////////////////////////////////////////////////////////////////////SHARED CODE
        drive.setTarget(backdropTrussDone);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
            outtake.update();
        }
        intake.intakePosition5th(0);
        intake.setPower(1);
        drive.setTarget(backdropTrussSusCode);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
            outtake.update();
        }
        drive.setTarget(whiteStack);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
            outtake.update();
        }
        drive.setTarget(whiteIntake);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
            outtake.update();
        }
        waitms(600);
        intake.intakePosition4th(0);
        drive.setTarget(whiteIntakeShake);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
            outtake.update();
        }
        waitms(600);
        drive.setTarget(whiteIntakeShakeInOut);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
            outtake.update();
        }
        waitms(100);
        intake.intakePositionExtended();
        drive.setTarget(whiteIntakeShake);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
            outtake.update();
        }
        waitms(400);
        intake.transferPosition();
        drive.setTarget(backdropTruss);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
            outtake.update();
        }
        intake.setPower(-1);
        drive.setTarget(backdropTrussDone);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
            outtake.update();
        }
        intake.setPower(0);
        intake.stay(0);
        outtake.setPixelLatch(true);
        waitms(100);
        outtake.depositPosition(150);
        drive.setTarget(new WayPoint(new Pose2d(53.5, 28.5, Rotation2d.fromDegrees(180)), 0.5));
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            outtake.update();
            intake.update();
        }
        waitms(600);
        outtake.setPixelLatch(false);
        waitms(800);
        outtake.transferPosition();
        drive.setTarget(backdropTrussDone);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
            outtake.update();
        }
        intake.intakePosition(0);
        intake.setPower(1);
        drive.setTarget(backdropTrussSusCode);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
            outtake.update();
        }
        drive.setTarget(whiteIntakeShakeSec);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
            outtake.update();
        }
        waitms(500);
        drive.setTarget(whiteIntakeShakeInOut);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
            outtake.update();
        }
        waitms(100);
        drive.setTarget(whiteIntakeShakeSecFarther);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
            outtake.update();
        }
        waitms(400);
        intake.transferPosition();
        drive.setTarget(backdropTruss);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
            outtake.update();
        }
        intake.setPower(-1);
        drive.setTarget(backdropTrussDone);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
            outtake.update();
        }
        intake.setPower(0);
        intake.stay(0);
        outtake.setPixelLatch(true);
        waitms(100);
        outtake.depositPosition(200);
        drive.setTarget(new WayPoint(new Pose2d(53.5, 28.5, Rotation2d.fromDegrees(180)), 0.5));
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            outtake.update();
            intake.update();
        }
        waitms(600);
        outtake.setPixelLatch(false);
        waitms(800);
        outtake.transferPosition();
        waitms(300);
    }
    public void waitms(long ms){
        ElapsedTime timer=new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds()<ms){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
            outtake.update();
        }

    }
}
