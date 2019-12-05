 /* Copyright (c) 2019 FIRST. All rights reserved.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
public class AutonomousOpMode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Servo arm = null;
    final static float rotationsPerInchForwardBackward=0;
    final static float rotationsPerInchStrafing=0;
    final static float rotationsPerDegreeRightLeft=0;
    final static float maxPower=0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftRear  = hardwareMap.get(DcMotor.class, "left_rear");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        arm = hardwareMap.get(Servo.class, "arm");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }
    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        /*
        * Code so that left stick goes forward backward and strafes
        * Right stick used to rotate
        */

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

   /*position 0 is right at the back and 5 is the closest to the skybridge. 
this function assumes viewforia already found both block positions and assumes the robot starts in the corner that has the 6 skystones lined up. It also assumes the robot starts facing the blocks*/
    void autonomous1(int blockPos1,int blockPost2){
      if(blockPos1>blockPos2){
       blockPos1=temp;
       blockPos1=blockPos2;
       blockPos2=blockPos1;
      }
      forwardBackward(30);
      strafe(blockPos1*7);
      pickUpBlock();
      strafe((5 - blockPos1)*7+40);
      releaseBlock();
      //now go back to the second block
      strafe((blockPos2 - 5)*7-40);
      pickUpBlock();
      //now go past the line and release the block
      strafe((5-blockPos2)*7+40);
      releaseBlock();
      //parcs the robot
      strafe(-15);
    }

    void forwardBackward(float inches){

    leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftFront.setTargetPosition(inches*rotationsPerInchForwardBackward);

    leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftRear.setTargetPosition(inches*rotationsPerInchForwardBackward);

    rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightFront.setTargetPosition(inches*rotationsPerInchForwardBackward);

    rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightRear.setTargetPosition(inches*rotationsPerInchForwardBackward);

    allOn();
    while(leftFront.isBusy() && opModeIsActive()) {
    }
    allOff();
}
void turn(float degrees){

    leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftFront.setTargetPosition(degrees*rotationsPerDegreeRightLeft);

    leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftRear.setTargetPosition(degrees*rotationsPerDegreeRightLeft);

    rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightFront.setTargetPosition(-degrees*rotationsPerDegreeRightLeft);

    rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightRear.setTargetPosition(-degrees*rotationsPerDegreeRightLeft);

    allOn();
    while(leftFront.isBusy() && opModeIsActive()) {
    }
    allOff();
}
void strafe(float inches){
    leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftFront.setTargetPosition(inches*rotationsPerInchStrafing);

    leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftRear.setTargetPosition(-inches*rotationsPerInchStrafing);

    rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightFront.setTargetPosition(inches*rotationsPerInchStrafing);

    rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightRear.setTargetPosition(-inches*rotationsPerInchStrafing);

    allOn();
    while(leftFront.isBusy() && opModeIsActive()) {
    }
    allOff();
}
void allOn(){
    leftFront.setPower(maxPower);
    leftRear.setPower(maxPower);
    rightRear.setPower(maxPower);
    rightFront.setPower(maxPower);
}
void allOff(){
    leftFront.setPower(0);
    leftRear.setPower(0);
    rightRear.setPower(0);
    rightFront.setPower(0);
}
void autonomous2(){
 driveArm();
 park();
}
//moves to the object grabs the object, moves backward then releases the object
void driveArm(){
 forwardBackward(40);
 //grabs the platform
 arm.setPosition(180);
 forwardBackward(-40);
 //releases the platform
 arm.setPosition(0);
 
}
//parks the car in autonomous at the end
void park(){
 //might be -90
 turn(90);
 //might be -60 depending on if you use 90 or -90
 forwardBackward(60);
}
}

