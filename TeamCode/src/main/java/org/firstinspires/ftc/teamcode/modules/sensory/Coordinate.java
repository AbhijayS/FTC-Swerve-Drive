package org.firstinspires.ftc.teamcode.modules.sensory;


public class Coordinate {

    private double x,y;

    /**
     * Creates a Coordinate Object to use in Odometry module
     * @param x
     * @param y
     */
    public Coordinate(double x, double y){
        this.x = x;
        this.y = y;
    }

    public void setXY(double x, double y){
        this.x = x;
        this.y = y;
    }

    public  void setX(double x){
        this.x = x;
    }

    public  void setY(double y){
        this.y = y;
    }

    public double getX(){
        return this.x;
    }

    public double getY(){
        return this.y;
    }

    public Coordinate getDifference(Coordinate a){
       Coordinate difference = new Coordinate(0,0);
       difference.setXY(a.getX()-this.x,a.getY()-this.y);
       return difference;
    }
}
