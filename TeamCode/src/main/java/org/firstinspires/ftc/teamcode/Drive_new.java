package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;


import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.io.FileWriter;
import java.nio.file.Files;

@TeleOp(name = "_Driver_new_", group = "Pushbot")
// @Disabled
public class Drive_new extends LinearOpMode {

    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware

    static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.

    static final double HEADING_THRESHOLD = 15;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable
    public static final float COUNTS_PER_INCH = 1;
    public ElapsedTime period  = new ElapsedTime();
    public static double UP_POSITION = 100;
    public static double DOWN_POSITION = 0;



    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("angles = ", robot.angles);    //
        telemetry.update();

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();


        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();


        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.update();
        }

        gyroDrive();

    }

    public void gyroDrive() {

        int globalX = 0;
        int globalY = 0;

        double max;
        double error;

        double speedToLeft;
        double speedToRight;
        double speedTurn = 0;

        double speedFrontLeft;
        double speedBackLeft;
        double speedFrontRight;
        double speedbackRight;


        double k = 0.1;
        double speedKof = 0.5;
        double startPoint = 0;

        double angle_now = 45;


        double mode = 0;

        if (opModeIsActive()) {

            robot.move(0, 0);

            while (opModeIsActive()) {
                robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                globalY += robot.getPositionRB();
                telemetry.addData("Position y:", globalY);

                double Y = gamepad1.left_stick_y;
                double X = -gamepad1.left_stick_x;
                boolean kofLeft = gamepad1.left_bumper;
                boolean kofRight = gamepad1.right_bumper;
                boolean qwerty = gamepad1.a;
                double leftTrigger = -gamepad1.left_trigger;
                double rightTrigger = gamepad1.right_trigger;

                if (qwerty) {
                    startPoint = robot.angles.firstAngle;
                }

                speedKof = settingKof(kofLeft, kofRight, speedKof);
                angle_now = getError(angle_now - (rightTrigger + leftTrigger));

                Y = drive(Y, k);
                X = drive(X, k);
                speedTurn = -(leftTrigger + rightTrigger);
                error = getError(robot.angles.firstAngle - startPoint + 45);
//                if ((Math.abs(error) == error) == (Math.abs(angle_now) == angle_now)) {
//                    speedTurn = Math.pow(-1, angle_now > error) * (Math.max(angle_now, error) - Math.min(angle_now, error));
//                }
//                if (angle_now > 0) {
//                    if (angle_now - error > 180 || angle_now - error < 0) {
//                        speedTurn = Math.abs(getError(angle_now - error)) / 180 * 1.5;
//                    }
//                    else {
//                        speedTurn = -Math.abs(getError(angle_now - error)) / 180 * 1.5;
//                    }
//                }
//                else {
//                    if (error - angle_now > 180 || error - angle_now < 0) {
//                        speedTurn = -Math.abs(getError(angle_now - error)) / 180 * 1.5;
//                    }
//                    else {
//                        speedTurn = Math.abs(getError(angle_now - error)) / 180 * 1.5;
//                    }
//                }
//

                double Y1 = Y * Math.sin(error * Math.PI / 180) * Math.sqrt(2) + X * Math.cos(error * Math.PI / 180) * Math.sqrt(2);
                double X1 = Y * Math.cos(error * Math.PI / 180) * Math.sqrt(2) - X * Math.sin(error * Math.PI / 180) * Math.sqrt(2);
                speedToLeft = X1;
                speedToRight = Y1;

                max = Math.max(Math.abs(speedToLeft), Math.abs(speedToRight));
                if (max > 1.0) {
                    speedToLeft /= max;
                    speedToRight /= max;
                }
                speedToLeft = speedNorm(speedKof, speedToLeft);
                speedToRight = speedNorm(speedKof, speedToRight);
                speedTurn = drive(speedTurn, k) * speedKof;


                speedFrontLeft = speedTurn + speedToRight;
                speedBackLeft = speedTurn + speedToLeft;
                speedFrontRight = -speedTurn + speedToLeft;
                speedbackRight = -speedTurn + speedToRight;

                max = Math.max(Math.abs(speedFrontLeft), Math.max(Math.abs(speedFrontRight), Math.max(Math.abs(speedbackRight), Math.abs(speedBackLeft))));
                if (max > 1.0) {
                    speedFrontLeft /= max;
                    speedBackLeft /= max;
                    speedFrontRight /= max;
                    speedbackRight /= max;
                }

                robot.move(speedFrontLeft, speedFrontRight, speedBackLeft, speedbackRight);


                double manipulatorUp = gamepad2.right_trigger;
                double manipulatorDown = gamepad2.left_trigger;
                boolean manipulatorAutoUp = gamepad2.dpad_up;
                boolean manipulatorAutoDown = gamepad2.dpad_down;
                boolean turnLeft = gamepad2.left_bumper;
                boolean turnRight = gamepad2.right_bumper;
                boolean clawOpen = gamepad1.dpad_left;
                boolean clawClose = gamepad1.dpad_right;
                double upperUpDown = gamepad1.right_stick_y;
                double upfordown = gamepad1.left_stick_y;

                if (clawClose && !clawOpen) {
                    robot.claw.setPosition(0.4);
                } else if (!clawClose && clawOpen) {
                    robot.claw.setPosition(0.75);
                }

                if (turnLeft && !turnRight) {
                    robot.turn.setPosition(1);
                } else if (!turnLeft && turnRight) {
                    robot.turn.setPosition(0);
                }
                if (manipulatorAutoUp) {
                    encoderDrive(1, UP_POSITION, 10, robot.upper);
                }
                else if (manipulatorAutoDown) {
                    encoderDrive(1, DOWN_POSITION, 10, robot.upper);
                }

                manipulatorUp = drive(manipulatorUp, k);
                manipulatorDown = drive(manipulatorDown, k);
                upperUpDown = drive(upperUpDown, k);
                upfordown = drive(-upfordown, k);

                if (robot.position >= 0.5 && upfordown < 0 || robot.position <= 0.8 && upfordown > 0) {
                    robot.position += (upfordown / 70);
                    robot.updown.setPosition(robot.position);
                }
                double manipulatorSpeed = manipulatorUp - manipulatorDown;

                robot.upper.setPower(-upperUpDown * 0.5);
                robot.manipulator.setPower(manipulatorSpeed);

                telemetry.addData("Speed", "%5.2f:%5.2f", speedToLeft, speedToRight);
                telemetry.addData("Y, X", "%5.2f:%5.2f", Y, X);
                telemetry.addData("Y1, X1", "%5.2f:%5.2f", Y1, X1);
                telemetry.addData("speedFrontLeft, speedFrontRight", "%5.2f:%5.2f", speedFrontLeft, speedFrontRight);
                telemetry.addData("speedBackLeft, speedbackRight", "%5.2f:%5.2f", speedBackLeft, speedbackRight);
                telemetry.addData("SpeedTurn", speedTurn);
                telemetry.addData("angles = ", robot.angles);
                telemetry.addData("angles first = ", robot.angles.firstAngle);
                telemetry.addData("SpeedR, SpeedL", "%5.2f:%5.2f", speedTurn + speedToLeft, -speedTurn + speedToRight);
                telemetry.addData("mode", mode);
//                telemetry.addData("left", robot.leftServo.getPosition());
//                telemetry.addData("right", robot.rightServo.getPosition());
                telemetry.addData("getError", getError(angle_now + 180));
                telemetry.addData("angle_now", angle_now);
                telemetry.addData("error", error);
                telemetry.update();
            }
            robot.move(0, 0);

        }
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double speedNorm(double koef, double speed) {
        return Range.clip(speed, -1, 1) * koef;
    }

    public double settingKof(boolean kofLeft, boolean kofRight, double speed) {
        if (kofLeft && !kofRight) {
            speed = 0.5;
        } else if (!kofLeft && kofRight) {
            speed = 0.75;
        }
        return speed;
    }

    public double drive(double gp, double zona) {
        if (gp >= -zona && gp <= zona) {
            gp = 0;
        } else if (gp > 0) {
            gp = -(gp - zona) / (1 - zona);
        } else {
            gp = (-gp - zona) / (1 - zona);
        }
        return gp;
    }

//    public void povorot(double mode, double startPoint, boolean plus, boolean minus, double first, boolean zero) {
//        double power1 = 0.5;
//        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        double error = getError(robot.angles.firstAngle - startPoint);
//        double error1 = getError(robot.angles.firstAngle - startPoint + 90);
//        double error2 = getError(robot.angles.firstAngle - startPoint - 90);
//        if (zero) {
//            while (opModeIsActive() && !(error <= 10 && error >= -10) && !gamepad1.dpad_down) {
//                double power;
//                double a = error;
//                if (-180 <= a && a <= 0) {
//                    power = power1;
//                }
//                else {
//                    power = -power1;
//                }
//                robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                error = getError(robot.angles.firstAngle - startPoint);
//
//                robot.move(-power, power, -power, power);
//
//                telemetry.addData("error", error);
//                telemetry.update();
//            }
//        }
//
//        if (plus && !minus) {
//
//            if (mode == 0) {
//                while (opModeIsActive() && (error <= 80 || error >= 100) && !gamepad1.dpad_down) {
//                    double power;
//                    double a = error;
//                    if (-90 <= a && a <= 90) {
//                        power = power1;
//                    }
//                    else {
//                        power = -power1;
//                    }
//                    robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                    error = getError(robot.angles.firstAngle - startPoint);
//
//                    robot.move(-power, power, -power, power);
//
//                    telemetry.addData("error", error);
//                    telemetry.update();
//                }
//            }
//            else {
//                while (opModeIsActive() && (error <= getError(error1 - 10) || error >= getError(error1 + 10)) && !gamepad1.dpad_down) {
//                    double power = power1;
//                    robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                    error = getError(robot.angles.firstAngle - startPoint);
//
//                    robot.move(-power, power, -power, power);
//
//                    telemetry.addData("error", error);
//                    telemetry.addData("error1", error1);
//                    telemetry.update();
//                }
//            }
//        }
//        if (!plus && minus) {
//            if (mode == 0) {
//                while (opModeIsActive() && (error >= -80 || error <= -100) && !gamepad1.dpad_down) {
//                    double power = 0;
//                    double a = error;
//                    if (-90 <= a && a <= 90) {
//                        power = -power1;
//                    }
//                    else {
//                        power = power1;
//                    }
//                    robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                    error = getError(robot.angles.firstAngle - startPoint);
//
//                    robot.move(-power, power, -power, power);
//
//                    telemetry.addData("error", error);
//                    telemetry.update();
//                }
//            }
//            else {
//                while (opModeIsActive() && (error <= getError(error2 - 10) || error >= getError(error2 + 10)) && !gamepad1.dpad_down) {
//                    double power = power1;
//                    double a = error;
//                    robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                    error = getError(robot.angles.firstAngle - startPoint);
//
//                    robot.move(power, -power, power, -power);
//
//                    telemetry.addData("error", error);
//                    telemetry.addData("error1", error2);
//                    telemetry.update();
//                }
//            }
//        }
//
//        robot.move(0, 0);
//
//    }
public void encoderDrive(double speed,
                         double inches,
                         double timeoutS, DcMotor motor) {
    int newTarget;

    // Determine new target position, and pass to motor controller
    newTarget = motor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
    motor.setTargetPosition(newTarget);

    // Turn On RUN_TO_POSITION
    motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    // reset the timeout time and start motion.
    period.reset();
    motor.setPower(Math.abs(speed));

    while (opModeIsActive() &&
            (period.seconds() < timeoutS) &&
            (motor.isBusy())) {

        // Display it for the driver.
        telemetry.addData("Running to",  " %7d :%7d", newTarget);
        telemetry.addData("Currently at",  " at %7d :%7d",
                motor.getCurrentPosition(), motor.getCurrentPosition());
        telemetry.update();
    }

    // Stop all motion;
    motor.setPower(0);

    // Turn off RUN_TO_POSITION
    motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    sleep(250);
}

}

