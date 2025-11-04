package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    public double kp, ki, kd, setpoint;
    private double previousLoopError = 0;
    private double integral = 0;
    private ElapsedTime time;
    private double lastOutput = 0;

    public PIDController(double kp, double ki, double kd){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.time = new ElapsedTime();
    }

    public PIDController(double kp, double ki, double kd, double setpoint){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        time = new ElapsedTime();
        this.setpoint = setpoint;
    }

    public double calculatePID(double currentValue, double setpoint){
        double dt = time.milliseconds();
        if (dt == 0) return this.lastOutput;
        double error = setpoint - currentValue;
        this.integral = integral + error * dt;
        double derivative = (error-previousLoopError)/dt;
        double output = (this.kp * error) + (this.ki * integral) + (this.kd * derivative);
        time.reset();
        previousLoopError = error;
        this.lastOutput = output;
        return output;
    }

    public double calculatePID(double currentValue){
        return calculatePID(currentValue, this.setpoint);
    }

    public void setSetpoint(double setpoint){ this.setpoint = setpoint; }
}
