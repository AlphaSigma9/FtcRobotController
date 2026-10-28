package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

//A = 2500 RPM
//B = 3000 RPM
//X = 0 RPM
@TeleOp(name = "Shooter Test", group = "Shooter")
public class flyWheel extends LinearOpMode {


    static final double TICKS_PER_REV = 28.0;
    static final double GEAR_RATIO    = 1.0;


    double kP = 0.008;
    double kI = 0.0002;
    double kD = 0.0;
    double kF = 0.75;
    double INTEGRAL_LIMIT = 0.5;

    @Override
    public void runOpMode() {

        DcMotorEx flyWheel = (DcMotorEx) hardwareMap.get(DcMotor.class, "FW");

        flyWheel.setDirection(DcMotor.Direction.FORWARD);
        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        ElapsedTime loopTime = new ElapsedTime();
        double targetRPM = 0;

        double integral = 0;
        double lastError = 0;
        double lastPos = flyWheel.getCurrentPosition(); //d position
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad1.a) targetRPM = 2500;
            if (gamepad1.b) targetRPM = 3000;
            if (gamepad1.x) { flyWheel.setPower(0); targetRPM = 0; break; }


            double dt = loopTime.seconds();  // dt
            loopTime.reset();

            // calculating velocity
            double posNow    = flyWheel.getCurrentPosition();
            double deltaTick = posNow - lastPos;
            lastPos = posNow;
            // trans to RPM
            double ticksPerSec = deltaTick / dt;
            double currentRPM  = ticksPerSec * 60.0 / TICKS_PER_REV / GEAR_RATIO;

            //Calculating error
            double error = targetRPM - currentRPM;

            //Add up to get power
            double ff = kF * (targetRPM / 6000.0);
            double p  = kP * error;

            integral = clamp(integral + error * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
            double i = kI * integral;

            double d = dt > 0 ? kD * ((error - lastError) / dt) : 0;
            lastError = error;

            double power = clamp(ff + p + i + d , 0.0, 1.0);
            flyWheel.setPower(power);


            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Actual RPM  (%.0f)", currentRPM);
            telemetry.addData("Error", error);
            telemetry.addData("dt", dt);
            telemetry.addData("deltaTick", deltaTick);
            telemetry.addData("Power", power);
            telemetry.update();
        }
        flyWheel.setPower(0);
    }

    double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
