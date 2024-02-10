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

@TeleOp(name="_Driver_", group="Pushbot")
// @Disabled
public class Drive extends LinearOpMode {

    HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware

    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 15 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable


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

    public void gyroDrive (){


        double  max;
        double  error;

        double speedToLeft;
        double speedToRight;
        double speedTurn = 0;

        double speedFrontLeft;
        double speedBackLeft;
        double speedFrontRight;
        double speedbackRight;


        double k = 0.2;
        double speedKof = 0.5;
        double startPoint = 0;

        double angle_now = 0;


        double mode = 0;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // start motion.

            robot.move(0, 0);

            while (opModeIsActive()) {
                robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                double Y = gamepad1.left_stick_y;
                double X = -gamepad1.left_stick_x;
                boolean kofLeft = gamepad1.left_bumper;
                boolean kofRight = gamepad1.right_bumper;
//                boolean MinusReturn = gamepad1.b;
//                boolean PlusReturn = gamepad1.x;
//                boolean ZeroReturn = gamepad1.y;
//                boolean kMode = gamepad1.dpad_up;
                boolean qwerty = gamepad1.a;
                double leftTrigger = -gamepad1.left_trigger;
                double rightTrigger = gamepad1.right_trigger;

//                if (kMode) {
//                    mode = (mode + 1) % 2;
//                }
//                povorot(mode, startPoint, PlusReturn, MinusReturn, robot.angles.firstAngle, ZeroReturn);

                if (qwerty) {
                    startPoint = robot.angles.firstAngle;
                }

                speedKof = settingKof(kofLeft, kofRight, speedKof);



                speedTurn = -(leftTrigger + rightTrigger);
                Y = drive(Y, k);
                X = drive(X, k);
                error = getError(robot.angles.firstAngle - startPoint + 45);

                double Y1 = Y * Math.sin(error * Math.PI / 180) * Math.sqrt(2) + X * Math.cos(error * Math.PI / 180) * Math.sqrt(2);
                double X1 = Y * Math.cos(error * Math.PI / 180) * Math.sqrt(2) - X * Math.sin(error * Math.PI / 180) * Math.sqrt(2);
                speedToLeft = X1;
                speedToRight = Y1;

                max = Math.max(Math.abs(speedToLeft), Math.abs(speedToRight));
                if (max > 1.0)
                {
                    speedToLeft /= max;
                    speedToRight /= max;
                }
                speedToLeft = speedNorm(speedKof, speedToLeft);
                speedToRight = speedNorm(speedKof, speedToRight);
                speedTurn = drive(speedTurn, k) * speedKof;


                speedFrontLeft = speedTurn + speedToRight;
                speedBackLeft = speedTurn + speedToLeft;
                speedFrontRight = -speedTurn + speedToLeft;
                speedbackRight = -speedTurn +  speedToRight;

                max = Math.max(Math.abs(speedFrontLeft), Math.max(Math.abs(speedFrontRight), Math.max(Math.abs(speedbackRight), Math.abs(speedBackLeft))));
                if (max > 1.0)
                {
                    speedFrontLeft /= max;
                    speedBackLeft /= max;
                    speedFrontRight /= max;
                    speedbackRight /= max;
                }

                robot.move(speedFrontLeft, speedFrontRight, speedBackLeft, speedbackRight);


                double manipulatorUp = gamepad2.right_trigger;
                double manipulatorDown = gamepad2.left_trigger;
                boolean turnLeft = gamepad2.left_bumper;
                boolean turnRight = gamepad2.right_bumper;
                boolean clawOpen = gamepad2.dpad_left;
                boolean clawClose = gamepad2.dpad_right;
                double upperUpDown = gamepad2.right_stick_y;

                if (clawClose && !clawOpen) {
                    robot.claw.setPosition(0.8);
                } else if (!clawClose && clawOpen) {
                    robot.claw.setPosition(0.5);
                }

                if (turnLeft && !turnRight) {
                    robot.turn.setPosition(0.9);
                } else if (!turnLeft && turnRight) {
                    robot.turn.setPosition(0);
                }

                manipulatorUp = drive(manipulatorUp, k);
                manipulatorDown = drive(manipulatorDown, k);
                upperUpDown = drive(upperUpDown, k);

                double manipulatorSpeed = manipulatorUp - manipulatorDown;

                robot.upper.setPower(-upperUpDown);
                robot.manipulator.setPower(manipulatorSpeed);

                telemetry.addData("Speed",   "%5.2f:%5.2f",  speedToLeft, speedToRight);
                telemetry.addData("Y, X",   "%5.2f:%5.2f",  Y, X);
                telemetry.addData("Y1, X1",   "%5.2f:%5.2f",  Y1, X1);
                telemetry.addData("SpeedTurn",  speedTurn);
                telemetry.addData("angles = ", robot.angles);
                telemetry.addData("angles first = ", robot.angles.firstAngle);
                telemetry.addData("error", error);
                telemetry.addData("SpeedR, SpeedL",   "%5.2f:%5.2f",  speedTurn + speedToLeft, -speedTurn + speedToRight);
                telemetry.addData("mode", mode);
//                telemetry.addData("left", robot.leftServo.getPosition());
//                telemetry.addData("right", robot.rightServo.getPosition());
                telemetry.addData("mode", mode);
                telemetry.update();
            }

            robot.move(0, 0);
        }
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double speedNorm(double koef, double speed) {
        return Range.clip(speed, -1, 1) * koef;
    }

    public double settingKof(boolean kofLeft, boolean kofRight, double speed) {
        if (kofLeft && !kofRight) {
            speed = 0.5;
        }
        else if (!kofLeft && kofRight) {
            speed = 1;
        }
        return speed;
    }

    public double drive(double gp, double zona) {
        if (gp >= -zona && gp <= zona) {
            gp = 0;
        }
        else if (gp > 0) {
            gp = -(gp - zona) / (1 - zona);
        }
        else {
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

}

