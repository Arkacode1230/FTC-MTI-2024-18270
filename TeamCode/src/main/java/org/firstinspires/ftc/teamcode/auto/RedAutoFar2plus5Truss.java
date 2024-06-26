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
import org.firstinspires.ftc.teamcode.auto.notshown.PropPosition;
import org.firstinspires.ftc.teamcode.auto.notshown.RedPipeline;
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
public class RedAutoFar2plus5Truss extends LinearOpMode {
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
        RedPipeline pipeline = new RedPipeline(telemetry, ObjectDirection);
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
                randomization=PropPosition.RIGHT;
            }
            else if (Objects.equals(pipeline.getPosition(), "MIDDLE")){
                telemetry.addData("Position", "MIDDLEE");
                randomization=PropPosition.MIDDLE;
            }
            else if (Objects.equals(pipeline.getPosition(), "RIGHT")){
                telemetry.addData("Position", "RIGHTO");
                randomization=PropPosition.LEFT;
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

        drive.setPositionEstimate(new Pose2d(-36.11, -62.16, Rotation2d.fromDegrees(90)));
        WayPoint rightPurpleWaypoint = new WayPoint(new Pose2d(-48, -40, Rotation2d.fromDegrees(90)), 1);
        WayPoint leftYellowWaypoint = new WayPoint(new Pose2d(50, -33, Rotation2d.fromDegrees(180)), 1);
        WayPoint middlePurpleWaypoint = new WayPoint(new Pose2d(-38, -38, Rotation2d.fromDegrees(90)), 1);
        WayPoint middleYellowWaypoint = new WayPoint(new Pose2d(50, -26, Rotation2d.fromDegrees(180)), 1);
        WayPoint whiteStack = new WayPoint(new Pose2d(-42, -44, Rotation2d.fromDegrees(160)), 3);
        WayPoint whiteIntake = new WayPoint(new Pose2d(-48, -39, Rotation2d.fromDegrees(160)), 1);
        WayPoint leftPurpleWaypoint = new WayPoint(new Pose2d(-41, -33, Rotation2d.fromDegrees(33)), 1);
        WayPoint rightYellowWaypoint = new WayPoint(new Pose2d(50, -19, Rotation2d.fromDegrees(180)), 1);
        WayPoint backdropTruss = new WayPoint(new Pose2d(-30, -50.5, Rotation2d.fromDegrees(180)), 1);
        WayPoint backdropTrussDone = new WayPoint(new Pose2d(35, -49, Rotation2d.fromDegrees(180)), 1);
        WayPoint backdropTrussSusCode = new WayPoint(new Pose2d(-17, -50, Rotation2d.fromDegrees(180)), 1);
        WayPoint whiteIntakeShake = new WayPoint(new Pose2d(-46, -35, Rotation2d.fromDegrees(160)), 1);
        WayPoint whiteIntakeShakeSec = new WayPoint(new Pose2d(-47, -30, Rotation2d.fromDegrees(160)), 1);

        telemetry.addData("Randomization", randomization.toString());
        telemetry.update();
        intake.transferPosition();
        intake.setTarget(50);
        outtake.setPixelLatch(false);
        intake.setPower(0.5);
        if (randomization==PropPosition.RIGHT) {
            drive.setTarget(rightPurpleWaypoint);

            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
            waitms(300);
            drive.setTarget(whiteStack);
            intake.stay(0);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
            intake.intakePosition5th();
            intake.setPower(1);
            drive.setTarget(whiteIntake);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
            waitms(700);
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
            waitms(200);
            outtake.setPixelLatch(true);
            waitms(200);
            outtake.depositPosition(150);
            drive.setTarget(rightYellowWaypoint);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                outtake.update();
                intake.update();
            }
            waitms(800);
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
                outtake.update();
            }
            drive.setTarget(whiteStack);
            intake.stay(0);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
            intake.intakePosition5th();
            intake.setPower(1);
            drive.setTarget(whiteIntake);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
            waitms(500);
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
            waitms(200);
            outtake.setPixelLatch(true);
            waitms(200);
            outtake.depositPosition(150);
            drive.setTarget(middleYellowWaypoint);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                outtake.update();
                intake.update();
            }
            waitms(800);
            outtake.setPixelLatch(false);
            waitms(500);
            outtake.transferPosition();
        }
        if (randomization==PropPosition.LEFT) {
            drive.setTarget(leftPurpleWaypoint);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
            drive.setTarget(whiteStack);
            intake.stay(0);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
            intake.intakePosition5th();
            intake.setPower(1);
            drive.setTarget(whiteIntake);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
            waitms(500);
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
            waitms(200);
            outtake.setPixelLatch(true);
            waitms(200);
            outtake.depositPosition(150);
            drive.setTarget(leftYellowWaypoint);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                outtake.update();
                intake.update();
            }
            waitms(800);
            outtake.setPixelLatch(false);
            waitms(500);
            outtake.transferPosition();
        }
        drive.setTarget(backdropTrussDone);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
            outtake.update();
        }
        intake.intakePositionExtended(0);
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
        waitms(400);
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
        outtake.depositPosition(200);
        drive.setTarget(middleYellowWaypoint);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            outtake.update();
            intake.update();
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
        intake.intakePositionExtended(0);
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
        drive.setTarget(middleYellowWaypoint);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            outtake.update();
            intake.update();
        }
        waitms(300);
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
