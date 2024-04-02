package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Drive extends LinearOpMode {

    public float lastAngle = 0;

    public boolean autoMod = false;

    double error;

    double speedTurn = 0;

    double k = 0.1;
    double speedKof = 0.5;
    int isHolder = 0;
    boolean holderPress = false;

    boolean isAutoDriveMod = false;

    @Override
    public void runOpMode() {}

    public void gyroDrive(double Y, double X, boolean kofLeft, boolean kofRight, boolean qwerty,
                          double leftTrigger, double rightTrigger, boolean holder,
                          boolean manipulatorAutoUp, boolean manipulatorAutoDown, boolean clawOpen,
                          boolean clawClose, double upperUpDown, boolean reInitEncoder,
                          boolean setAutoGo, boolean exitBorder, HardwarePushbot robot) {
        boolean isReInitUpPos = (reInitEncoder && manipulatorAutoUp);
        boolean isReInitDownPos = (reInitEncoder && manipulatorAutoDown);
        if (isReInitUpPos){
            manipulatorAutoUp = false;
        }
        if (isReInitDownPos){
            manipulatorAutoDown = false;
        }
        if (setAutoGo){
            isAutoDriveMod = true;
            robot.setMoveTo(robot.getPositionLF() + 1000);
        }
        else if ((Y != 0) || (X != 0) || leftTrigger != 0 || rightTrigger != 0) {
            isAutoDriveMod = false;
        }
        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if (qwerty) {
            robot.changeStartAngle();
        }
        if (isAutoDriveMod){
            robot.updateAutoMove();
        }
        else {
            speedKof = settingKof(kofLeft, kofRight, speedKof);
            Y = checkZone(Y, k);
            X = checkZone(X, k);

            speedTurn = -(leftTrigger + rightTrigger);
            if (Math.abs(lastAngle - robot.angles.firstAngle) > 0.01 && speedTurn == 0){
                speedTurn = (lastAngle - robot.angles.firstAngle) / Math.abs(lastAngle - robot.angles.firstAngle) * 0.2;
            }
            speedTurn = checkZone(speedTurn, k) * speedKof;
            robot.Move(speedKof, Y, -X, speedTurn);
        }

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
            robot.moveUpper(upperUpDown, exitBorder);
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
        telemetry.addData("targetM", robot.targetMoveRobot);
        telemetry.addData("distanceC", (robot.targetMoveRobot - robot.getPositionLF()));

        telemetry.update();
        lastAngle = robot.angles.firstAngle;

    }

    public double settingKof(boolean kofLeft, boolean kofRight, double speed) {
        if (kofLeft && !kofRight) {
            speed = 0.5;
        } else if (!kofLeft && kofRight) {
            speed = 0.95;
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

