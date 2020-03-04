/* Copyright (c) 2019 FIRST. All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to
 imitations in the disclaimer below) provided that
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
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name = "testauto", group = "Iterative Opmode")
public class MechBotAutoDemo extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Declare all electronics being used
    private DcMotor leftRear;
    private DcMotor leftFront;
    private DcMotor rightRear;
    private DcMotor rightFront;
    private DcMotor spinLeft = null;
    private DcMotor spinRight = null;
    private Servo leftPlat;
    private Servo rightPlat;
    private Servo grabServo = null;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables.
        // The name in quotes must be the same as what is on the phone configuration
        // This is done to tell the code that the variable "leftRear" for example...
        // ...corisponds with a real motor on the robot
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        spinLeft = hardwareMap.get(DcMotor.class, "spin_left");
        spinRight = hardwareMap.get(DcMotor.class, "spin_right");
        grabServo = hardwareMap.get(Servo.class, "grab_servo");
        leftPlat = hardwareMap.servo.get("left_plat");
        rightPlat = hardwareMap.servo.get("right_plat");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        spinLeft.setDirection(DcMotor.Direction.FORWARD);
        spinRight.setDirection(DcMotor.Direction.REVERSE);
        
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        grabOn(true);
        grabServo.setPosition(1);
        waitForStart();

        
        //Code goes here


        telemetry.addData("Status", "Initialized");
    }
    
    /* Documentation:
    
        grabOn(boolean on) - changes the state of the platform grabber (true is grabbing, false is open)
        spinOn(double power) - set the power of the intake (1 is in, -1 is out, 0 is off) 

    For all of the rest, the "speed" is how fast it is going (from 0 to 1) and inches is the encoder 
    position it runs to (not actual inches 600 is about 1 foot)

        turnRight(double speed, double inches)
        turnLeft(double speed, double inches)

        leftStrafe(float speed,float degrees)
        rightStrafe(float speed,float degrees)

        backward(double speed, double inches)
        forward(double speed, double inches)
    
    */


    void grabOn (boolean on) {
        if (on){
            leftPlat.setPosition(0);
            rightPlat.setPosition(1);
        } else {
            leftPlat.setPosition(0.9);
            rightPlat.setPosition(0.1);
        }
    }

    void spinOn(double power) {
        spinLeft.setPower(power);
        spinRight.setPower(power);
    }
    
    void turnRight(double speed, double inches) {
        speedMax/=2;
        speedMin/=2;

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        leftRear.setPower(speed);
        leftFront.setPower(speed);
        rightRear.setPower(-speed);
        rightFront.setPower(-speed);
        double total=leftRear.getCurrentPosition()+inches;
        double start=leftRear.getCurrentPosition();
        while(leftRear.getCurrentPosition()<total){
            double inertiaCancel=1-Math.abs(start-leftRear.getCurrentPosition())/Math.abs(total);
            if(inertiaCancel<0.02){
                break;
            }
            telemetry.addData("percent complete",1-Math.abs(start-leftRear.getCurrentPosition())/Math.abs(total));
            if(Math.abs(speedMax*inertiaCancel)<speedMin){
                telemetry.addData("percent complete",1-Math.abs(start-leftRear.getCurrentPosition())/Math.abs(total));

                leftRear.setPower(speed);
                leftFront.setPower(speed);
                rightRear.setPower(-speed);
                rightFront.setPower(-speed);
            }else {
                leftRear.setPower(speed * inertiaCancel);
                leftFront.setPower(speed * inertiaCancel);
                rightRear.setPower(-speed * inertiaCancel);
                rightFront.setPower(-speed * inertiaCancel);
            }
            telemetry.update();

        }
        allOff();
    }

    void turnLeft(double speed, double inches) {
        speedMax/=2;
        speedMin/=2;
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftRear.setPower(-speed);
        leftFront.setPower(-speed);
        rightRear.setPower(speed);
        rightFront.setPower(speed);

        double total=leftRear.getCurrentPosition()+inches;
        double start=leftRear.getCurrentPosition();

        while(leftRear.getCurrentPosition()<total){
            double inertiaCancel=1-Math.abs(start-leftRear.getCurrentPosition())/Math.abs(total);
            if(inertiaCancel<0.02){
                break;
            }
            telemetry.addData("percent complete",1-Math.abs(start-leftRear.getCurrentPosition())/Math.abs(total));
            if(Math.abs(speedMax*inertiaCancel)<speedMin){
                telemetry.addData("percent complete",1-Math.abs(start-leftRear.getCurrentPosition())/Math.abs(total));

                leftRear.setPower(-speed);
                leftFront.setPower(-speed);
                rightRear.setPower(speed);
                rightFront.setPower(speed);
            }else {
                leftRear.setPower(-speed);
                leftFront.setPower(-speed);
                rightRear.setPower(speed);
                rightFront.setPower(speed);
            }
            telemetry.update();

        }
        allOff();
    }


    void rightStrafe(float speed,float degrees){
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

       leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


       leftRear.setPower(-speed);
       leftFront.setPower(speed);
       rightRear.setPower(speed);
       rightFront.setPower(-speed);
       double temp=rightFront.getCurrentPosition();
       double total=leftRear.getCurrentPosition()+degrees;
       double start=leftRear.getCurrentPosition();
       while(leftRear.getCurrentPosition()<total){
           double inertiaCancel=1-Math.abs(start-leftRear.getCurrentPosition())/Math.abs(total);
           if(inertiaCancel<0.02){
               break;
           }
           telemetry.addData("percent complet2e",1-Math.abs(start-leftRear.getCurrentPosition())/Math.abs(total));
           if(Math.abs(speedMax*inertiaCancel)<speedMin){
               telemetry.addData("percent complete",1-Math.abs(start-leftRear.getCurrentPosition())/Math.abs(total));

               leftRear.setPower(-speed);
               leftFront.setPower(speed);
               rightRear.setPower(speed);
               rightFront.setPower(-speed);
   
           }else {
               inertiaCancel=1;
               
               
              leftRear.setPower(0);
              leftFront.setPower(0);
              rightRear.setPower(0);
              rightFront.setPower(0);
           }
            telemetry.update(); 
       }
       allOff();
    }

    void backward(double speed, double inches){
        //choose 1 motor to do all the counting (this should stay consistent across all methods then reset that motors
        //encoders
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set all speeds to negative 1 for backward
        leftRear.setPower(-speed);
        leftFront.setPower(-speed);
        rightRear.setPower(-speed);
        rightFront.setPower(-speed);

        //declare variables for total target position and start position
        double total=leftRear.getCurrentPosition()-inches;
        double start=leftRear.getCurrentPosition();

        //this whileloop runs while the robot has not gone farther past the total
        while(leftRear.getCurrentPosition()>total){
            //the inertia cancel varaible calculates the percentage that your motion is complete
            double inertiaCancel=1-Math.abs(start-leftRear.getCurrentPosition())/Math.abs(total);

            //one it gets so low that the power would be negladble, stop the loop (it goes all the way to infinity if this
            //is not done
            if(inertiaCancel<0.02){
                break;
            }
            //check if we exceeded max speed
            if(Math.abs(speedMax*inertiaCancel)<speedMin){
                telemetry.addData("percent complete",1-Math.abs(start-leftRear.getCurrentPosition())/Math.abs(total));

                leftRear.setPower(-speed);
                leftFront.setPower(-speed);
                rightRear.setPower(-speed);
                rightFront.setPower(-speed);
            }else {
                leftRear.setPower(-speed * inertiaCancel);
                leftFront.setPower(-speed * inertiaCancel);
                rightRear.setPower(-speed * inertiaCancel);
                rightFront.setPower(-speed * inertiaCancel);
            }
            telemetry.update();

        }
        allOff();
    }


   void forward(double speed, double inches){
       leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       leftRear.setPower(speed);
       leftFront.setPower(speed);
       rightRear.setPower(speed);
       rightFront.setPower(speed);
       double total = leftRear.getCurrentPosition()+inches;
       double start = leftRear.getCurrentPosition();
       while(leftRear.getCurrentPosition()<total){
           double inertiaCancel=1-Math.abs(start-leftRear.getCurrentPosition())/Math.abs(total);
           if(inertiaCancel<0.02){
               break;
           }
           telemetry.addData("percent complet2e",1-Math.abs(start-leftRear.getCurrentPosition())/Math.abs(total));
           if(Math.abs(speedMax*inertiaCancel)<speedMin){
               telemetry.addData("percent complete",1-Math.abs(start-leftRear.getCurrentPosition())/Math.abs(total));

               leftRear.setPower(speed);
               leftFront.setPower(speed);
               rightRear.setPower(speed);
               rightFront.setPower(speed);
           }else {
               inertiaCancel=1;
               leftRear.setPower(speed * inertiaCancel);
               leftFront.setPower(speed * inertiaCancel);
               rightRear.setPower(speed * inertiaCancel);
               rightFront.setPower(speed * inertiaCancel);
           }
            telemetry.update();
       }
       allOff();
   }

    void allOff(){
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);
    }
    void leftStrafe(float speed,float degrees){
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

       leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


       leftRear.setPower(speed);
       leftFront.setPower(speed);
       rightRear.setPower(speed);
       rightFront.setPower(speed);
       double temp=rightFront.getCurrentPosition();
       
       double total=leftRear.getCurrentPosition()+degrees;
       double start=leftRear.getCurrentPosition();
       while(leftRear.getCurrentPosition()<total){
           double inertiaCancel=1-Math.abs(start-leftRear.getCurrentPosition())/Math.abs(total);
           if(inertiaCancel<0.02){
               break;
           }
           telemetry.addData("percent complet2e",1-Math.abs(start-leftRear.getCurrentPosition())/Math.abs(total));
           if(Math.abs(speedMax*inertiaCancel)<speedMin){
               telemetry.addData("percent complete",1-Math.abs(start-leftRear.getCurrentPosition())/Math.abs(total));

               leftRear.setPower(speed);
               leftFront.setPower(speed);
               rightRear.setPower(speed);
               rightFront.setPower(speed);
   
           }else {
               inertiaCancel=1;
               
               
              leftRear.setPower(0);
              leftFront.setPower(0);
              rightRear.setPower(0);
              rightFront.setPower(0);
           }
            telemetry.update(); 
       }
       allOff();
      
        
    }
   
    void allOnRight(){
        leftFront.setPower(-1);
        leftRear.setPower(-1);
        rightRear.setPower(1);
        rightFront.setPower(1);
    }
    void allOnLeft(){
        leftFront.setPower(1);
        leftRear.setPower(1);
        rightRear.setPower(-1);
        rightFront.setPower(-1);
    }
    void allOn(){
        leftFront.setPower(maxPower);
        leftRear.setPower(maxPower);
        rightRear.setPower(1);
        rightFront.setPower(maxPower);
    }
    void allOnForwardBack(){
        leftFront.setPower(maxPower);
        leftRear.setPower(maxPower);
        rightRear.setPower(.48);
        rightFront.setPower(.48);
    }
    
 
}