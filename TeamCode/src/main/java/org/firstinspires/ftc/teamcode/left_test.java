

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

@Autonomous(name="left_test_auto", group="Pushbot")
// @Disabled
public class left_test extends LinearOpMode {

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
            robot.move(-0.5, 0.5, 0.5, -0.5);
            sleep(1000);
            robot.move(0, 0);
//            autoDrive(0, -1, 2 * k, 4);
//            autoDrive(0.5, 0, 0.7 * k, 4);
//            upper(0.5, 1.1, robot.str);
//            sleep(100);
//            autoDrive(0, 0.5, 1.5 * k, 4);
//            sleep(100);
//            autoDrive(0.25, 0, 0.2 * k, 4);
//            manipulator(-1, 1);
//            autoDrive(-0.25, 0, 2 * k, 4);
//            Turn(0.3, 0);
//            autoDrive(0, -1, 4 * k, 4);
//            autoDrive(0.5, 0, k, 2);


        }
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    public void Turn(double speed, double angle) {
        double error = getError(robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) + 180;
        double A = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (opModeIsActive() && !((angle - 5 <= A && A <= angle + 5))) {
            A = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            double power = speed;
            if (angle >= error && angle - error <= 180 || angle <= error && error - angle >= 180) {
                power = -speed;
            }

            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            robot.move(-power, power, -power, power);

            telemetry.addData("error", (int) (-1 * COUNTS_PER_INCH));
            telemetry.addData("error", robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("error", robot.leftFront.getCurrentPosition());

            telemetry.update();
        }
    }

    public void autoDrive (double Y, double X, double distance, double time) {
        int     newLeftTarget;
        int     moveCounts;
        double  max;
        double  error;

        double speedToLeft;
        double speedToRight;


        if (opModeIsActive()) {
            robot.move(0, 0);

            robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = robot.leftFront.getCurrentPosition() + moveCounts;

            double r = getRuntime();

            while (opModeIsActive() && Math.abs(robot.leftFront.getCurrentPosition()) < Math.abs(newLeftTarget) && getRuntime() - r <= time) {
//                if (robot.str == null) {
//                    man.runOpMode(robot);
//                }
                robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                error = getError(robot.angles.firstAngle + 45);

                speedToRight = Y * Math.sin(error * Math.PI / 180) * Math.sqrt(2) - X * Math.cos(error * Math.PI / 180) * Math.sqrt(2);
                speedToLeft = Y * Math.cos(error * Math.PI / 180) * Math.sqrt(2) + X * Math.sin(error * Math.PI / 180) * Math.sqrt(2);

                max = Math.max(Math.abs(speedToLeft), Math.abs(speedToRight));
                if (max > 1.0)
                {
                    speedToLeft /= max;
                    speedToRight /= max;
                }

                robot.move(speedToRight, speedToLeft);

                telemetry.addData("Speed",   "%5.2f:%5.2f",  speedToLeft, speedToRight);
                telemetry.addData("Y, X",   "%5.2f:%5.2f",  Y, X);
                telemetry.addData("angles = ", robot.angles);
                telemetry.addData("angles first = ", robot.angles.firstAngle);
                telemetry.addData("error", error);
                telemetry.addData("count", COUNTS_PER_INCH);
                telemetry.addData("target", newLeftTarget);
                telemetry.addData("pos", robot.leftFront.getCurrentPosition());
                telemetry.addData("time", getRuntime());
                telemetry.update();

            }
            robot.move(0, 0);
            sleep(100);
        }
    }

//    public void manipulator(double power, double time) {
//        double a = getRuntime();
//        while (opModeIsActive() && (getRuntime() - (a) <= time)) {
//            robot.leftServo.setPosition(-power + robot.leftKoef);
//            robot.rightServo.setPosition(power + robot.rightKoef);
//            telemetry.addData("123", a);
//            telemetry.addData("сейчас", getRuntime() - (a));
//            telemetry.addData("time", time);
//            telemetry.update();
//        }
//        robot.leftServo.setPosition(robot.leftKoef);
//        robot.rightServo.setPosition(robot.rightKoef);
//    }

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
