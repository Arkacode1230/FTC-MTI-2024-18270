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

@Autonomous
@Config
public class RedAutoFarStageDoor2plus5 extends LinearOpMode {
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
            ////////////////////////////////////////////////////////////DETECTION
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
        ////////////////////////////////////////////////Defining waypoints
        waitForStart();
        outtake.resetEncoder();
        intake.resetEncoder();
        drive.setPositionEstimate(new Pose2d(-36.11, -62.16, Rotation2d.fromDegrees(90)));
        WayPoint rightPurpleWaypoint = new WayPoint(new Pose2d(-48, -41, Rotation2d.fromDegrees(90)), 1);
        WayPoint leftYellowWaypoint = new WayPoint(new Pose2d(49.5, -33, Rotation2d.fromDegrees(180)), 1);
        WayPoint middlePurpleWaypoint = new WayPoint(new Pose2d(-35, -38, Rotation2d.fromDegrees(90)), 1);
        WayPoint middleYellowWaypoint = new WayPoint(new Pose2d(49.5, -29, Rotation2d.fromDegrees(180)), 1);
        WayPoint whiteStack = new WayPoint(new Pose2d(-42, -35, Rotation2d.fromDegrees(180)), 2);
        WayPoint whiteIntake = new WayPoint(new Pose2d(-46.9, -5, Rotation2d.fromDegrees(180)), 1);
        WayPoint whiteIntakeRight = new WayPoint(new Pose2d(-29, -5, Rotation2d.fromDegrees(180)), 1);
        WayPoint leftPurpleWaypoint = new WayPoint(new Pose2d(-41, -33, Rotation2d.fromDegrees(33)), 1);
        WayPoint rightYellowWaypoint = new WayPoint(new Pose2d(49.5, -19, Rotation2d.fromDegrees(180)), 1);
        WayPoint backdropTruss = new WayPoint(new Pose2d(-30, -4, Rotation2d.fromDegrees(180)), 1);
        WayPoint backdropTrussDone = new WayPoint(new Pose2d(40, -4, Rotation2d.fromDegrees(180)), 1);
        WayPoint backdropTrussExtend = new WayPoint(new Pose2d(-18.5, -4, Rotation2d.fromDegrees(180)), 1);
        WayPoint Paaark = new WayPoint(new Pose2d(52, -6, Rotation2d.fromDegrees(180)), 1);
        intake.transferPosition();
        intake.setTarget(50);
        outtake.setPixelLatch(false);
        intake.setPower(0.5);
        if (randomization==PropPosition.RIGHT) { ////////////////////////////LEFT RANDOMIZATION
            drive.setTarget(rightPurpleWaypoint);

            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
            drive.setTarget(new WayPoint(new Pose2d(-37.71, -47.33, Rotation2d.fromDegrees(90)), 2));
            intake.stay(0);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
            drive.setTarget(new WayPoint(new Pose2d(-29, -10, Rotation2d.fromDegrees(90)), 2));
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }

            drive.setTarget(whiteIntakeRight);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
            intake.intakePosition4th(520);
            intake.setPower(1);
            waitms(1700);
            intake.transferPosition();
            drive.setTarget(backdropTruss);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
            waitms(800);
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
        if (randomization==PropPosition.MIDDLE) { /////////////////////////////////MIDDLE RANDOMIZATION
            drive.setTarget(middlePurpleWaypoint);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
            drive.setTarget(new WayPoint(new Pose2d(-36, -44, Rotation2d.fromDegrees(-270)), 4));
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
            drive.setTarget(whiteIntake);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
            intake.intakePosition4th();
            intake.setPower(1);
            waitms(1000);
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
        if (randomization==PropPosition.LEFT) {//////////////////////////////LEFT RANDOMIZATION
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
            drive.setTarget(whiteIntake);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
            intake.intakePosition4th();
            intake.setPower(1);
            waitms(1000);
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


        //////////////////////////////////////////////////////CYCLE CODE
        drive.setTarget(backdropTrussDone);
        //back though stagedoor
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
            outtake.update();
        }
        intake.intakePosition2nd(940);
        intake.setPower(1);
        //Extend
        drive.setTarget(backdropTrussExtend);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
            outtake.update();
        }
        //Intake
        waitms(400);
        intake.setTarget(600);
        waitms(400);
        intake.intakePositionExtended(900);
        //IN AND OUT THING
        waitms(600);
        intake.transferPosition();
        waitms(1200);
        intake.setPower(-1);
        drive.setTarget(backdropTrussDone);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
            outtake.update();
        }
        //Back to backdrop
        intake.setPower(0);
        intake.stay(0);
        outtake.setPixelLatch(true);
        waitms(100);
        outtake.depositPosition(200);
        drive.setTarget(middleYellowWaypoint);
        //drop in middle
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
        intake.intakePositionExtended(940);
        intake.setPower(1);
        //Extend 2nd time
        drive.setTarget(backdropTrussExtend);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
            outtake.update();
        }
        waitms(400);
        intake.setTarget(700);
        waitms(400);
        intake.setTarget(940);
        //IN AND OUT THING
        waitms(600);
        intake.transferPosition();
        waitms(1300);
        intake.setPower(-1);
        //BAck to backdrop
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
            outtake.update();
        }
        waitms(300);
        outtake.setPixelLatch(false);
        waitms(400);
        outtake.transferPosition();
        waitms(300);
        //park
        drive.setTarget(Paaark);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
            outtake.update();
        }
        ///wahhoie all donese
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
