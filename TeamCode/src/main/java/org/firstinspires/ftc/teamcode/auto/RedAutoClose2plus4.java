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
public class RedAutoClose2plus4 extends LinearOpMode {
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
        drive.setPositionEstimate(new Pose2d(11.83, -62.16, Rotation2d.fromDegrees(90)));
        WayPoint leftPurpleWaypoint = new WayPoint(new Pose2d(26, -42, Rotation2d.fromDegrees(90)), 1);
        WayPoint leftYellowWaypoint = new WayPoint(new Pose2d(55, -36, Rotation2d.fromDegrees(180)), 1);
        WayPoint middlePurpleWaypoint = new WayPoint(new Pose2d(14, -37, Rotation2d.fromDegrees(90)), 0.5);
        WayPoint middleYellowWaypoint = new WayPoint(new Pose2d(55.5, -28.5, Rotation2d.fromDegrees(180)), 1);

        WayPoint rightCenterWaypoint = new WayPoint(new Pose2d(22, -32, Rotation2d.fromDegrees(-225)), 2);
        WayPoint rightPurpleWaypoint = new WayPoint(new Pose2d(20, -30, Rotation2d.fromDegrees(180)), 1);
        WayPoint rightYellowWaypoint = new WayPoint(new Pose2d(54.5, -23, Rotation2d.fromDegrees(180)), 1);

        intake.transferPosition();
        intake.setTarget(50);
        outtake.setPixelLatch(true);
        if (randomization==PropPosition.LEFT) {
            drive.setTarget(leftPurpleWaypoint);

            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
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
            drive.setTarget(new WayPoint(new Pose2d(43, -6, Rotation2d.fromDegrees(180)), 2));
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
        }
        if (randomization==PropPosition.MIDDLE) {
            drive.setTarget(middlePurpleWaypoint);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
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
            drive.setTarget(new WayPoint(new Pose2d(35, -6, Rotation2d.fromDegrees(180)), 2));
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
        }
        if (randomization==PropPosition.RIGHT) {
            drive.setTarget(rightCenterWaypoint);
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
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
            drive.setTarget(new WayPoint(new Pose2d(30, -6, Rotation2d.fromDegrees(180)), 2));
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
        }

        drive.setTarget(new WayPoint(new Pose2d(-11, -3.5, Rotation2d.fromDegrees(-178.6)), 0.5));//to intake from stacks
        intake.intakePosition5th(980);
        intake.setPower(1);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
            outtake.update();
        }
        intake.intakePosition4th();//change heights
        waitms(1000);
        intake.intakePositionExtended(900);
        waitms(500);
        intake.setTarget(1000);//back and forth
        waitms(1000);
        drive.setTarget(new WayPoint(new Pose2d(30, -6, Rotation2d.fromDegrees(180)), 2));//come back from stack intaking
        intake.stay(0);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
            outtake.update();
        }
        intake.transferPosition();
        waitms(500);
        intake.setPower(-1);//transfer sequence
        waitms(900);
        intake.setPower(0);
        drive.setTarget(new WayPoint(new Pose2d(51, -26, Rotation2d.fromDegrees(180)), 1));//to backdrop first time after intaking
        intake.stay(0);
        outtake.setPixelLatch(true);
        waitms(300);
        outtake.depositPosition(300);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            outtake.update();
            intake.update();
            outtake.update();
        }
        waitms(1000);
        outtake.setPixelLatch(false);//release 1st cycle
        waitms(1000);
        outtake.transferPosition();
        if (randomization==PropPosition.LEFT) {//position of intermediate point changes, so we need these if statements
            drive.setTarget(new WayPoint(new Pose2d(43, -6, Rotation2d.fromDegrees(180)), 2));
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
        }
        if (randomization==PropPosition.MIDDLE) {
            drive.setTarget(new WayPoint(new Pose2d(35, -6, Rotation2d.fromDegrees(180)), 2));
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
        }
        if (randomization==PropPosition.RIGHT) {
            drive.setTarget(new WayPoint(new Pose2d(30, -6, Rotation2d.fromDegrees(180)), 2));
            while (!drive.atTarget() && opModeIsActive()){
                drive.updateLocalizer();
                drive.updatePIDS();
                intake.update();
                outtake.update();
            }
        }

        drive.setTarget(new WayPoint(new Pose2d(-12.5, -3.5, Rotation2d.fromDegrees(178.6)), 0.5));//intake from stack second time
        intake.intakePositionExtended(970);
        intake.setPower(1);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
            outtake.update();
        }
        waitms(200);
        intake.setTarget(800);//back and forth
        waitms(400);
        intake.setTarget(1000);
        waitms(900);
        drive.setTarget(new WayPoint(new Pose2d(30, -6, Rotation2d.fromDegrees(180)), 2));//going back to the backdrop 2nd time
        intake.stay(0);
        while (!drive.atTarget() && opModeIsActive()){
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
            outtake.update();
        }
        intake.transferPosition();
        waitms(500);
        intake.setPower(-1);//transfer sequence second time
        waitms(900);
        intake.setPower(0);
        drive.setTarget(new WayPoint(new Pose2d(51, -26, Rotation2d.fromDegrees(180)), 1));//to backdrop second time
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
            drive.updateLocalizer();
            drive.updatePIDS();
            intake.update();
            outtake.update();
        }
        
    }
}
