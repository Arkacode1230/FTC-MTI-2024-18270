package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.function.DoubleSupplier;
@Config
public class GyroLocalizer {

    private double prevLeftEncoder, prevRightEncoder, prevHorizontalEncoder;
    private Rotation2d previousAngle=new Rotation2d();
    private Rotation2d resetAngle=new Rotation2d();
    private double centerWheelOffset;
    public static double trackWidth = 10.779073867039818;
    public static double TICKS_TO_INCHES = 0.0029401084684705092;
    public static double CENTER_WHEEL_OFFSET = -6.95;
    IMU imu;
    double headingOffset=0;
    MotorEx leftEncoder, rightEncoder, perpEncoder;
    Pose2d robotPose=new Pose2d();

    public GyroLocalizer(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();
        leftEncoder=new MotorEx(hardwareMap, "backright");
        rightEncoder=new MotorEx(hardwareMap, "frontleft");
        perpEncoder=new MotorEx(hardwareMap, "frontright");
        leftEncoder.setDistancePerPulse(-TICKS_TO_INCHES);
        rightEncoder.setDistancePerPulse(TICKS_TO_INCHES);
        perpEncoder.setDistancePerPulse(TICKS_TO_INCHES);
        leftEncoder.resetEncoder();
        rightEncoder.resetEncoder();
        perpEncoder.resetEncoder();

    }

    /**
     * This handles all the calculations for you.
     */
    public void updatePose() {
        update(leftEncoder.getDistance(), rightEncoder.getDistance(), perpEncoder.getDistance());
    }

    public void updatePose(Pose2d pose) {
        previousAngle = pose.getRotation();
        resetAngle=pose.getRotation();
        robotPose = pose;

        prevLeftEncoder = 0;
        prevRightEncoder = 0;
        prevHorizontalEncoder = 0;
        imu.resetYaw();
        leftEncoder.resetEncoder();
        rightEncoder.resetEncoder();
        perpEncoder.resetEncoder();
    }

    public void update(double leftEncoderPos, double rightEncoderPos, double horizontalEncoderPos) {
        double deltaLeftEncoder = leftEncoderPos - prevLeftEncoder;
        double deltaRightEncoder = rightEncoderPos - prevRightEncoder;
        double deltaHorizontalEncoder = horizontalEncoderPos - prevHorizontalEncoder;

        Rotation2d angle = resetAngle.plus(
                new Rotation2d(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS))
        );

        prevLeftEncoder = leftEncoderPos;
        prevRightEncoder = rightEncoderPos;
        prevHorizontalEncoder = horizontalEncoderPos;

        double dw = (angle.minus(previousAngle).getRadians());

        double dx = (deltaLeftEncoder + deltaRightEncoder) / 2;
        double dy = deltaHorizontalEncoder - (centerWheelOffset * dw);

        Twist2d twist2d = new Twist2d(dx, dy, dw);

        Pose2d newPose = robotPose.exp(twist2d);

        previousAngle = angle;

        robotPose = new Pose2d(newPose.getTranslation(), angle);
    }
    public Pose2d getPose() {
        return robotPose;
    }
    public void rotatePose(double byAngle) {
        robotPose = robotPose.rotate(byAngle);
    }
}