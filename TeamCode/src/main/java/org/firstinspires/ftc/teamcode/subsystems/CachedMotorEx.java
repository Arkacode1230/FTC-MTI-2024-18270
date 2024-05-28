package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CachedMotorEx extends MotorEx {
    double prevSetVelocity=0;
    public CachedMotorEx(@NonNull HardwareMap hMap, String id) {
        this(hMap, id, GoBILDA.NONE);
    }
    public CachedMotorEx(@NonNull HardwareMap hMap, String id, @NonNull GoBILDA gobildaType) {
        super(hMap, id, gobildaType);
    }
    public CachedMotorEx(@NonNull HardwareMap hMap, String id, double cpr, double rpm) {
        super(hMap, id, cpr, rpm);
    }
    @Override
    public void set(double output) {
        setPower(output);
    }
    void setPower(double velo){
        if (velo==prevSetVelocity){
            return;
        }
        if (velo==0 || Math.abs(velo)==1){
            motorEx.setPower(velo);
            prevSetVelocity=velo;
            return;
        }
        if (Math.abs(velo-prevSetVelocity)>0.01){
            motorEx.setPower(velo);
            prevSetVelocity=velo;
        }
    }
}
