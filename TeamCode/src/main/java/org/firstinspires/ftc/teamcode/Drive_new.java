package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "DRIVERS_TWo", group = "Pushbot")
// @Disabled
public class Drive_new extends LinearOpMode {

    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    public static int UP_POSITION = -740;
    public static int DOWN_POSITION = 0;
    public static int HALF_POSITION = (UP_POSITION - DOWN_POSITION) / 2;
    public static int SCALES_POSITION = 30;
    public int newTarget = DOWN_POSITION;
    public int epsilont = 5;
    public float lastAngle = 0;

    public boolean autoMod = false;

    public double clawCloseCof = 0.4;
    public double clawOpenCof = 0.45;

    public double minSpeedUpper = 0.2;
    public double maxSpeedUpper = 1;
    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("angles = ", robot.angles);
        telemetry.update();
        telemetry.addData(">", "Calibrating Gyro");
        telemetry.update();


        telemetry.addData(">", "Robot Ready.");
        telemetry.update();

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
        int isHolder = 0;
        double holderStartPosition = 0;
        double holderEndPosition = 0.7;
        boolean holderPress = false;
        double speedUpper = minSpeedUpper;

        if (opModeIsActive()) {

            robot.move(0, 0);

            while (opModeIsActive()) {
                robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                double Y = gamepad1.left_stick_y;
                double X = -gamepad1.left_stick_x;
                boolean kofLeft = gamepad1.left_bumper;
                boolean kofRight = gamepad1.right_bumper;
                boolean qwerty = gamepad1.a;
                double leftTrigger = -gamepad1.left_trigger;
                double rightTrigger = gamepad1.right_trigger;
                boolean holder = gamepad2.x;
                boolean manipulatorAutoUp = gamepad2.dpad_up;
                boolean manipulatorAutoDown = gamepad2.dpad_down;
                boolean clawOpen = gamepad2.dpad_left;
                boolean clawClose = gamepad2.dpad_right;
                double upperUpDown = gamepad2.right_stick_y;
                boolean reInitEncoder = gamepad2.a;

                boolean isReInitUpPos = (reInitEncoder && manipulatorAutoUp);
                boolean isReInitDownPos = (reInitEncoder && manipulatorAutoDown);
                if (isReInitUpPos){
                    manipulatorAutoUp = false;
                }
                if (isReInitDownPos){
                    manipulatorAutoDown = false;
                }

                if (qwerty) {
                    startPoint = robot.angles.firstAngle;
                }

                speedKof = settingKof(kofLeft, kofRight, speedKof);
                angle_now = getError(angle_now - (rightTrigger + leftTrigger));

                Y = drive(Y, k);
                X = drive(X, k);
                speedTurn = -(leftTrigger + rightTrigger);
                /*if (Math.abs(lastAngle - robot.angles.firstAngle) > 0.01 && speedTurn == 0){
                    speedTurn = -(lastAngle - robot.angles.firstAngle) / Math.abs(lastAngle - robot.angles.firstAngle) * 0.2;
                }*/
                error = getError(robot.angles.firstAngle - startPoint + 45);

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


                if (holder && !holderPress){
                    holderPress = true;
                    if (isHolder == 0){
                        robot.holderLeft.setPosition(holderEndPosition);
                        robot.holderRight.setPosition(1 - holderEndPosition);
                        isHolder = 1;
                    }
                    else{
                        robot.holderLeft.setPosition(holderStartPosition);
                        robot.holderRight.setPosition(1 - holderStartPosition);
                        isHolder = 0;
                    }
                }
                else if (!holder){
                    holderPress = false;
                }

                if (clawClose && !clawOpen) {
                    robot.claw.setPosition(clawOpenCof);
                } else if (!clawClose && clawOpen) {
                    robot.claw.setPosition(clawCloseCof);
                }
                if (isReInitDownPos){
                    DOWN_POSITION = robot.upper.getCurrentPosition();
                    HALF_POSITION = (UP_POSITION - DOWN_POSITION) / 2;
                }
                if (isReInitUpPos){
                    UP_POSITION = robot.upper.getCurrentPosition();
                    HALF_POSITION = (UP_POSITION - DOWN_POSITION) / 2;
                }
                updateEncoder(robot.upper);
                if (Math.abs(upperUpDown) > 0.1){
                    autoMod = false;
                    speedUpper = maxSpeedUpper * Math.pow((1 - Math.abs(HALF_POSITION - robot.upper.getCurrentPosition()) / (HALF_POSITION * -1.0)), 1);
                    if (speedUpper < minSpeedUpper){
                        speedUpper = minSpeedUpper;
                    }
                    newTarget += 15 * upperUpDown;

                    setTargetMotor(speedUpper, newTarget, robot.upper);
                }
                else if (! autoMod){
                    newTarget = robot.upper.getCurrentPosition();
                    setTargetMotor(0.5, newTarget, robot.upper);
                }

                if (manipulatorAutoUp) {
                    autoMod = true;
                    setTargetMotor(0.2, UP_POSITION, robot.upper);
                }
                else if (manipulatorAutoDown) {
                    autoMod = true;
                    setTargetMotor(0.2, DOWN_POSITION, robot.upper);
                }

               /* telemetry.addData("Speed", "%5.2f:%5.2f", speedToLeft, speedToRight);
                telemetry.addData("Y, X", "%5.2f:%5.2f", Y, X);
                telemetry.addData("Y1, X1", "%5.2f:%5.2f", Y1, X1);
                telemetry.addData("speedFrontLeft, speedFrontRight", "%5.2f:%5.2f", speedFrontLeft, speedFrontRight);
                telemetry.addData("speedBackLeft, speedbackRight", "%5.2f:%5.2f", speedBackLeft, speedbackRight);
                telemetry.addData("SpeedTurn", speedTurn);
                telemetry.addData("angles = ", robot.angles);
                telemetry.addData("angles first = ", robot.angles.firstAngle);
                telemetry.addData("SpeedR, SpeedL", "%5.2f:%5.2f", speedTurn + speedToLeft, -speedTurn + speedToRight);
                telemetry.addData("mode", mode);
                telemetry.addData("getError", getError(angle_now + 180));
                telemetry.addData("angle_now", angle_now);
                telemetry.addData("error", error);*/
                telemetry.addData("UP position", "%7d", UP_POSITION);
                telemetry.addData("DOWN position", "%7d", DOWN_POSITION);
                telemetry.addData("HALF position", "%7d", HALF_POSITION);
                telemetry.addData("dd", "%6f", (1 - Math.abs(HALF_POSITION - robot.upper.getCurrentPosition()) / (HALF_POSITION * -1.0)));
                telemetry.addData("Speed upper", "%7f", speedUpper);
                telemetry.addData("Running to",  " %7d", newTarget);
                telemetry.addData("Current Position",  " %7d", robot.upper.getCurrentPosition());
                telemetry.addData("LEN", " %5f", robot.sensorRange.getDistance(DistanceUnit.MM));
                telemetry.addData("leftHolder", "%3f", robot.holderLeft.getPosition());
                telemetry.addData("rightHolder", "%3f", robot.holderRight.getPosition());
                telemetry.update();
                lastAngle = robot.angles.firstAngle;
            }
            robot.move(0, 0);

        }
    }

    public double getError(double targetAngle) {

        double robotError;
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

    public void setTargetMotor(double speed,
                               int target, DcMotor motor) {

        newTarget = target;
        motor.setTargetPosition(newTarget);

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(Math.abs(speed));

    }

    public void updateEncoder(DcMotor motor){
        if (!motor.isBusy()){
            if (Math.abs(newTarget - motor.getCurrentPosition()) > epsilont){
                setTargetMotor(0.2, newTarget, motor);
            }
            else{
                motor.setPower(0);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }


        }
    }
}

