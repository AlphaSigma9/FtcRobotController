package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Shooter — 飞轮闭环转速控制
 *
 * 支持两种方式：
 *  1) 内置速度环（推荐）：DcMotorEx.setVelocity() + RUN_USING_ENCODER，SDK 内部用 PIDF 闭环。
 *  2) 自写 PIDF：前馈 kF 提供基础油门，PID 只做修正，精度更高、便于针对大负载调参。
 *
 * 换算关系：
 *   ticks/sec = rpm * ticksPerRev * gearRatio / 60
 *   内嵌电机编码器：ticksPerRev = 28  (motion 解题 / 依型号，通常 28)
 */

public class shooter {

    // ---------- 参数区（按你的机器人调整） ----------
    private static final double TICKS_PER_REV   = 28.0;
    private static final double GEAR_RATIO      = 1.0;

    private static final int    VELOCITY_MODE   = 0;
    private double kP = 0.008;
    private double kI = 0.0002;
    private double kD = 0.0;
    private double kF = 0.75;                              // 前馈油门（0~1），先标定这一步
    private double integralLimit = 0.5;                    // 积分限幅，防发射时积分爆掉

    private DcMotorEx  motor;
    private ElapsedTime loopTime;
    private double     targetRPM;
    private double     integral, lastError;

    public shooter(HardwareMap hardwareMap, String motorName) {
        motor = (DcMotorEx) hardwareMap.get(DcMotor.class, motorName);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        loopTime = new ElapsedTime();
    }

    /** 启动飞轮并进入闭环速度模式。 */
    public void init(double targetRPM) {
        this.targetRPM = targetRPM;
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);   // 进入编码器闭环模式
        motor.setPower(0.0);                                // 重新触发模式切换
        loopTime.reset();
    }

    /**
     * 每 loop 调用一次，维持 targetRPM。
     * 必须持续调用：闭环指令只在每次调用时下发，不调用就不维持。
     */
    public void update() {
        switch (VELOCITY_MODE) {
            case 1:  updateSelfPID();  break;
            default: updateBuiltIn();  break;
        }
    }

    // ---------- 方式一：内置速度环 ----------
    private void updateBuiltIn() {
        // RUN_USING_ENCODER + setVelocity 的入参单位是 ticks/sec
        double ticksPerSec = rpmToTicksPerSecond(targetRPM);
        motor.setVelocity(ticksPerSec);
    }

    // ---------- 方式二：自写 PIDF ----------
    private void updateSelfPID() {
        double dt    = loopTime.seconds();
        loopTime.reset();

        double currentRPM = getRPM();
        double error = targetRPM - currentRPM;

        double ff = kF * (targetRPM / 6000.0);  // 以 6000 RPM 对应满油门为基准标定前馈

        double p = kP * error;

        integral += error * dt;
        integral = clamp(integral, -integralLimit, integralLimit);
        double i = kI * integral;

        double derivative = dt > 0 ? (error - lastError) / dt : 0.0;
        double d = kD * derivative;
        lastError = error;

        double power = clamp(ff + p + i + d, 0.0, 1.0);
        motor.setPower(power);
    }

    // ---------- 状态 / 反馈 ----------
    /** 当前实际转速（RPM），来自编码器速度。 */
    public double getRPM() {
        double ticksPerSec = motor.getVelocity();      // ticks/sec
        return ticksPerSec * 60.0 / TICKS_PER_REV / GEAR_RATIO;
    }

    /** 当前目标转速（RPM）。 */
    public double getTargetRPM() { return targetRPM; }

    /** 到达目标转速 ±tol 判定是否就绪（用于触发发射）。 */
    public boolean atSetpoint(double toleranceRPM) {
        return Math.abs(getRPM() - targetRPM) <= toleranceRPM;
    }

    /** 停止飞轮。 */
    public void stop() {
        motor.setPower(0.0);
    }

    // ---------- 工具 ----------
    private double rpmToTicksPerSecond(double rpm) {
        return rpm * TICKS_PER_REV * GEAR_RATIO / 60.0;
    }
    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    // ---------- 调参入口（可选） ----------
    public void setPIDF(double p, double i, double d, double f) {
        kP = p; kI = i; kD = d; kF = f;
    }
}