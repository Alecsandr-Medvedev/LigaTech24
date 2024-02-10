package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.HardwarePushbot.k;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

@Autonomous(name="RED_auto", group="Pushbot")
// @Disabled
public class autoRed extends LinearOpMode {

    HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
//    autonomusVuforia        man     = new autonomusVuforia();

    static final double     COUNTS_PER_MOTOR_REV    = 55; //1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.43;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 6.0;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private int a = 0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();
        while (a == 0) {
//            man.runOpMode(robot);
            telemetry.addData("picture", robot.str);
            telemetry.addData("picture", robot.vect);
            telemetry.addData("picture", robot.orint);
            telemetry.addData("123", 123);
            telemetry.addData("v", robot.v);
            telemetry.update();
            if (opModeIsActive()) {
                a = 1;
            }
        }
        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData(">", "-Robot Ready.");    //
        telemetry.update();

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        if (opModeIsActive()) {
            manipulator(-0.5, 5);
//            robot.move(-0.5, 0.5, 0.5, -0.5);
//            sleep(1000);
//            robot.move(0, 0);


        }
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void manipulator(double power, double time) {
        double a = getRuntime();
        robot.claw.setPosition(0.35);
        while (opModeIsActive() && (getRuntime() - (a) <= time)) {
            robot.upper.setPower(power);
            robot.manipulator.setPower(0.4);
            telemetry.addData("123", a);
            telemetry.addData("сейчас", getRuntime() - (a));
            telemetry.addData("time", time);
            telemetry.update();
        }
        robot.upper.setPower(0);
        robot.move(0.1, 0.3, 0.1, 0.1);
        sleep(700);
        robot.move(0, 0);
        robot.claw.setPosition(0.8);
    }

    public void upper(double power, double time, String k) {
        double a = getRuntime();
        if (k == null) {
            while (opModeIsActive() && (getRuntime() - (a) <= time)) {
                robot.manipulator.setPower(power);
                telemetry.addData("начало", a);
                telemetry.addData("сейчас", getRuntime() - (a));
                telemetry.addData("time", time);
                telemetry.update();
            }
        }
        else {
            double v = robot.vect.get(0);
            if (v <= 290) {
                upper(0.5, 3.3, null);
                // справа куб 3 уровень
            }
            else if (v >= 310) {
                upper(0.5, 1.1, null);
                // лева  1уровень
            }
            else {
                upper(0.5, 2.2, null);
                //центр 2 уроыень
            }
        }
        robot.manipulator.setPower(0);

    }

    public double getError(double targetAngle) {
        double robotError = targetAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

}
