package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Drive extends LinearOpMode {

    public float lastAngle = 0;

    public boolean autoMod = false;


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
    int isHolder = 0;
    boolean holderPress = false;

    @Override
    public void runOpMode() {}

    public void gyroDrive(double Y, double X, boolean kofLeft, boolean kofRight, boolean qwerty,
                          double leftTrigger, double rightTrigger, boolean holder,
                          boolean manipulatorAutoUp, boolean manipulatorAutoDown, boolean clawOpen,
                          boolean clawClose, double upperUpDown, boolean reInitEncoder, HardwarePushbot robot) {
        boolean isReInitUpPos = (reInitEncoder && manipulatorAutoUp);
        boolean isReInitDownPos = (reInitEncoder && manipulatorAutoDown);
        if (isReInitUpPos){
            manipulatorAutoUp = false;
        }
        if (isReInitDownPos){
            manipulatorAutoDown = false;
        }
        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if (qwerty) {
            startPoint = robot.angles.firstAngle;
        }

        move(X, Y, qwerty, kofLeft, kofRight, rightTrigger, leftTrigger, robot);
        speedKof = settingKof(kofLeft, kofRight, speedKof);
        robot.Move(speedKof);

        if (holder && !holderPress){
            holderPress = true;
            if (isHolder == 0){
                robot.downHolder();
                isHolder = 1;
            }
            else{
                robot.upHolder();
                isHolder = 0;
            }
        }
        else if (!holder){
            holderPress = false;
        }

        if (clawClose && !clawOpen) {
            robot.openClaw();
        } else if (!clawClose && clawOpen) {
            robot.closeClaw();
        }
        robot.setNewPositionUpper(isReInitDownPos, isReInitUpPos);
        robot.updateEncoder(robot.upper);

        if (Math.abs(upperUpDown) > 0.1){
            autoMod = false;
            robot.moveUpper(upperUpDown);
        }
        else if (! autoMod){
            robot.stopUpper();
        }

        if (manipulatorAutoUp) {
            autoMod = true;
            robot.upperAutoUp();
        }
        else if (manipulatorAutoDown) {
            autoMod = true;
            robot.upperAutoDown();
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
        telemetry.addData("UP position", "%7d", robot.UP_POSITION);
        telemetry.addData("DOWN position", "%7d", robot.DOWN_POSITION);
        telemetry.addData("HALF position", "%7d", robot.HALF_POSITION);
        telemetry.addData("dd", "%6f", (1 - Math.abs(robot.HALF_POSITION - robot.upper.getCurrentPosition()) / (robot.HALF_POSITION * -1.0)));
        telemetry.addData("Speed upper", "%7f", robot.speedUpper);
        telemetry.addData("Running to",  " %7d", robot.newTargetUpper);
        telemetry.addData("Current Position",  " %7d", robot.upper.getCurrentPosition());
        telemetry.addData("LEN", " %5f", robot.sensorRange.getDistance(DistanceUnit.MM));
        telemetry.addData("leftHolder", "%3f", robot.holderLeft.getPosition());
        telemetry.addData("rightHolder", "%3f", robot.holderRight.getPosition());
        telemetry.addData("positionC", robot.getPositionLF());
        telemetry.update();
        lastAngle = robot.angles.firstAngle;

    }

    public void move(double X, double Y, boolean qwerty, boolean kofLeft, boolean kofRight,
                     double rightTrigger, double leftTrigger, HardwarePushbot robot){

        Y = checkZone(Y, k);
        X = checkZone(X, k);
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
        speedTurn = checkZone(speedTurn, k) * speedKof;


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

    public double checkZone(double gp, double zona) {
        if (gp >= -zona && gp <= zona) {
            gp = 0;
        } else if (gp > 0) {
            gp = -(gp - zona) / (1 - zona);
        } else {
            gp = (-gp - zona) / (1 - zona);
        }
        return gp;
    }
}

