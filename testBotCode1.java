package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TestBotCode1", group = "")
public class TestBotCode1 extends LinearOpMode {

  private DcMotor left_drive;
  private DcMotor right_drive;
  private Servo servo1;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    double tgtPower;
    double tgtPosition;

    left_drive = hardwareMap.dcMotor.get("left_drive");
    right_drive = hardwareMap.dcMotor.get("right_drive");
    servo1 = hardwareMap.servo.get("servo1");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        // LEFT MOTOR FORWARD
        tgtPower = gamepad1.right_stick_y;
        left_drive.setPower(tgtPower);
        telemetry.update();
        telemetry.addData("Target Power", tgtPower);
        telemetry.addData("Motor Power", left_drive.getPower());
        // RIGHT MOTOR FORWARD
        tgtPower = -gamepad1.right_stick_y;
        right_drive.setPower(tgtPower);
        telemetry.update();
        telemetry.addData("Target Power", tgtPower);
        telemetry.addData("Motor Power", right_drive.getPower());
        // LEFT TURN
        tgtPower = gamepad1.right_stick_x;
        left_drive.setPower(tgtPower);
        telemetry.update();
        telemetry.addData("Target Power", tgtPower);
        telemetry.addData("Motor Power", right_drive.getPower());
        // RIGHT TURN
        tgtPower = gamepad1.right_stick_x;
        right_drive.setPower(tgtPower);
        telemetry.update();
        telemetry.addData("Target Power", tgtPower);
        telemetry.addData("Motor Power", left_drive.getPower());
        // SERVO
        
        //tgtPower = -gamepad1.left_stick_y;
        //servo1.setPosition(tgtPower);
        boolean left_bumper= gamepad1.left_bumper;
        //tgtPosition = -gamepad1.left_bumper;
        if(left_bumper){
         servo1.setPosition(30);
        }
        else if(left_bumper == false){
          servo1.setPosition(0);
        }
        //tgtPosition = tgtPosition + 1;
        //tgtPosition = tgtPosition / 2;
       // servo1.setPosition(tgtPosition);
        //servo1.setPosition(30);
      //  telemetry.addData("Target Position", tgtPosition);
        telemetry.addData("Servo Position", servo1.getPosition());
        telemetry.update();
       
      }
    }
  }
}
  
