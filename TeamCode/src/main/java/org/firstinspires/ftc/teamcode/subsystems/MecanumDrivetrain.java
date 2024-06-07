package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pathing.WayPoint;

import java.util.Arrays;
import java.util.List;
@Config
public class MecanumDrivetrain {
    MotorEx leftFront;
    MotorEx leftBack;
    MotorEx rightFront;
    MotorEx rightBack;
    SimpleMotorFeedforward forwardFeedforward=new SimpleMotorFeedforward(0.085, 1);
    SimpleMotorFeedforward strafeFeedforward=new SimpleMotorFeedforward(0.22, 1);
    SimpleMotorFeedforward headingFeedforward=new SimpleMotorFeedforward(0.115, 1);

    PIDFController translationalControllerY=new PIDFController(0.1, 0, 0.008, 0);
    PIDFController translationalControllerX=new PIDFController(
            translationalControllerY.getP(),
            translationalControllerY.getI(),
            translationalControllerY.getD(),
            translationalControllerY.getF());
    PIDFController headingController=new PIDFController(1, 0, 0, 0);


    public GyroLocalizer localizer;
    Telemetry telemetry;
    FtcDashboard dashboard;

    public void init(HardwareMap hwMap, Telemetry telemetry, FtcDashboard dashboard){
        this.telemetry=telemetry;
        this.dashboard=dashboard;
        leftFront=new MotorEx(hwMap, "frontleft");
        leftBack=new MotorEx(hwMap, "backleft");
        rightFront=new MotorEx(hwMap, "frontright");
        rightBack=new MotorEx(hwMap, "backright");

        localizer = new GyroLocalizer(hwMap);
    }
    public void setPositionEstimate(Pose2d position){

        localizer.updatePose(position);
    }
    public void setRawPowers(double frontleft, double frontright, double backleft, double backright){
        double maximum=Math.max(frontleft, frontright);
        maximum=Math.max(maximum, backleft);
        maximum=Math.max(maximum, backright);
        if (maximum>1){
            frontleft=frontleft/maximum;
            frontright=frontright/maximum;
            backleft=backleft/maximum;
            backright=backright/maximum;

        }
        leftFront.set(-frontleft);
        leftBack.set(-backleft);
        rightFront.set(frontright);
        rightBack.set(backright);
    }
    public void setWeightedPowers(double front, double strafe, double heading){
        double weightedFront=forwardFeedforward.calculate(front);
        double weightedStrafe=strafeFeedforward.calculate(strafe);
        double weightedHeading=headingFeedforward.calculate(heading);

        setRawPowers(
                (weightedFront - weightedStrafe - weightedHeading),
                (weightedFront + weightedStrafe + weightedHeading),
                (weightedFront + weightedStrafe - weightedHeading),
                (weightedFront - weightedStrafe + weightedHeading)
                );
    }

    public void driveFieldCentric(double XPower, double YPower, double turnPower, double currHeading){
        double x = XPower * Math.cos(currHeading) + YPower * Math.sin(currHeading);
        double y = YPower * Math.cos(currHeading) - XPower * Math.sin(currHeading);
        setWeightedPowers(x, y, turnPower);
    }
    public void setTarget(WayPoint target){
        translationalControllerX.setSetPoint(target.getPosition().getX());
        translationalControllerY.setSetPoint(target.getPosition().getY());
        headingController.setSetPoint(target.getPosition().getRotation().getRadians());

        translationalControllerX.setTolerance(target.getTolerance().getTranslation().getX());
        translationalControllerY.setTolerance(target.getTolerance().getTranslation().getY());
        headingController.setTolerance(target.getTolerance().getRotation().getRadians());
    }
    public void updateLocalizer() {
        localizer.updatePose();
        Pose2d position=localizer.getPose();
        telemetry.addLine(position.toString());
        TelemetryPacket packet = new TelemetryPacket();


        packet.fieldOverlay().setFill("blue")
                .strokeCircle(position.getX(), position.getY(), 9)
                .strokeLine(position.getX(), position.getY(),
                        (position.getRotation().getCos()*10)+ position.getX(),
                        (position.getRotation().getSin()*10)+ position.getY());

        dashboard.sendTelemetryPacket(packet);
    }
    public List<Double> getVelocity(){
        List<Double> returning=Arrays.asList(rightBack.getVelocity(),
                leftFront.getVelocity(),
                rightFront.getVelocity());

        return returning;
    }
    public void updatePIDS(){
        double heading=localizer.getPose().getRotation().getRadians();
        while (Math.abs(heading-headingController.getSetPoint())>Math.PI){
            if (heading<headingController.getSetPoint()){
                heading=heading+2*Math.PI;
            }else{
                heading=heading-2*Math.PI;
            }
        }
        double x_velo=translationalControllerX.calculate(localizer.getPose().getX());
        double y_velo=translationalControllerY.calculate(localizer.getPose().getY());
        double heading_velo=headingController.calculate(heading);
        telemetry.addData("velocity x", x_velo);
        telemetry.addData("velocity y", y_velo);
        telemetry.addData("velocity heading", heading_velo);

        if (translationalControllerY.atSetPoint()){
            y_velo=0;
        }
        if (translationalControllerX.atSetPoint()){
            x_velo=0;
        }
        if (headingController.atSetPoint()){
            heading_velo=0;
        }


        driveFieldCentric(x_velo, y_velo,heading_velo, heading);
    }
    public boolean atTarget(){
        return translationalControllerX.atSetPoint() && translationalControllerY.atSetPoint() && headingController.atSetPoint();
    }
}
