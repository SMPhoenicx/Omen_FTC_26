package org.firstinspires.ftc.teamcode.unused;

import com.qualcomm.robotcore.hardware.Servo;
import java.util.ArrayList;
public class ToggleServo{
    private Servo servo;
    private ArrayList<Integer> angles = new ArrayList<>();
    private ArrayList<Servo.Direction> dirs = new ArrayList<>();
    private int anglesLength;
    private int dirsLength;
    public int pos;
    public ToggleServo(Servo s, int[] states, Servo.Direction direction){
        this.servo = s;
        this.anglesLength = states.length;
        for(int i = 0; i < this.anglesLength; i++){
            this.angles.add(states[i]);
        }
        this.pos = 0;
        this.servo.setDirection(direction);
        this.servo.setPosition(angles.get(0) / 355.0);
    }
    public ToggleServo(Servo s, int[] states, Servo.Direction[] directions, Servo.Direction startDir, int startState){
        this.servo = s;
        this.anglesLength = states.length;
        this.dirsLength = directions.length;
        for(int i = 0; i < this.anglesLength; i++){
            this.angles.add(states[i]);
        }
        for(int i = 0; i < this.dirsLength; i++){
            this.dirs.add(directions[i]);
        }
        this.pos = 0;
        this.servo.setDirection(startDir);
        this.servo.setPosition(startState / 355.0);
    }

    public ToggleServo(Servo s, int[] states, Servo.Direction direction, int startState){
        this.servo = s;
        this.anglesLength = states.length;
        for(int i = 0; i < this.anglesLength; i++){
            this.angles.add(states[i]);
        }
        this.pos = 0;
        this.servo.setDirection(direction);
        this.servo.setPosition(startState / 355.0);
    }
    public void toggle(){
        this.pos++;
        if(this.pos == this.anglesLength) this.pos = 0;
        this.servo.setPosition(angles.get(this.pos) / 355.0);
    }
    public void toggleLeft(){
        if(this.pos > 0) {
            this.pos--;
        }
        this.servo.setPosition(angles.get(this.pos) / 355.0);
    }

    public void toggleRight(){
        if(this.pos < this.anglesLength - 1) {
            this.pos++;
        }
        this.servo.setPosition(angles.get(this.pos) / 355.0);
    }
    public void toggleLeftWithDir(){
        if(this.pos > 0) {
            this.pos--;
        }
        servo.setDirection(dirs.get(this.pos));
        this.servo.setPosition(angles.get(this.pos) / 355.0);
    }
    public void toggleRightWithDir(){
        servo.setDirection(dirs.get(this.pos));
        if(this.pos < this.anglesLength - 1) {
            this.pos++;
        }
        if(this.pos < this.dirsLength - 1) {
            servo.setDirection(dirs.get(this.pos));
        }
        this.servo.setPosition(angles.get(this.pos) / 355.0);
    }

    public void setIndex(int i) {
        this.servo.setPosition(angles.get(i) / 355.0);
    }

    public Servo getServo(){return this.servo;}
}