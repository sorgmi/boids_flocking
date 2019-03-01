package boids;

import android.util.Log;

import dalvik.annotation.TestTarget;

/**
 * A minimal Vector class for Java
 */

public class Vektor {

    public double x = -1;
    public double y = -1;
    private double z = -1;
    public boolean threeD = false;


    public Vektor(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vektor(double x, double y, double z) {
        threeD = true;
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vektor(Vektor v) {
        this.x = v.getX();
        this.y = v.getY();
        if (v.isThreeD()) {
            this.z = v.getZ();
            this.threeD = true;
        }
    }

    public Vektor(double x1, double y1, double x2, double y2) {
        this.x = x2 - x1;
        this.y = y2 - y1;
    }

    public Vektor(double x1, double y1, double z1, double x2, double y2, double z2) {
        this.x = x2 - x1;
        this.y = y2 - y1;
        this.z = z2 - z1;
        threeD = true;
    }

    /**
     * returns the Vector AB (from A to B)
     * @param a
     * @param b
     * @return
     */
    public static Vektor subtract(Vektor a, Vektor b) {
        double xx = b.getX() - a.getX();
        double yy = b.getY() - a.getY();
        // System.out.println(yy);
        if (a.threeD && b.threeD) {
            double zz = b.getZ() - a.getZ();
            return new Vektor(xx, yy, zz);
        } else if (a.threeD ^ b.threeD) {
            Log.wtf("dlr", "Class Vektor: Subtract 2D and 3D Vektor. Will return a 2D Vektor!");
        }

        return new Vektor(xx, yy);
    }

    public void normalize() {
        double lenght = this.getLenght();
        if(lenght == 0) this.multiply(0);
        else this.multiply(1 / lenght);
    }


    public void multiply(double factor) {
        x = factor * x;
        y = factor * y;
        if (threeD) z = factor * z;
    }

    public void add(Vektor v) {
        x += v.getX();
        y += v.getY();
        if (threeD && v.isThreeD()) {
            z += v.getZ();
        }
    }

    public void sub(Vektor v) {
        x -= v.getX();
        y -= v.getY();
        if (threeD && v.isThreeD()) {
            z -= v.getZ();
        }
    }

    public void limit(double max){
        if(this.getLenght() > max){
            this.normalize();
            this.multiply(max);
        }
    }

    /**
     * May not working correct!
     *
     * @deprecated use {@link #subtract(Vektor, Vektor)} and {@link #getLenght()} instead.
     */
    @Deprecated
    public static double distance(Vektor a, Vektor b){
        double d1 = b.getX() - a.getX() * b.getX() - a.getX();
        double d2 = b.getY() - a.getY() * b.getY() - a.getY();
        if(a.threeD && b.threeD){
            double d3 = b.getZ() - a.getZ() * b.getZ() - a.getZ();
            return Math.sqrt(d1+d2+d3);
        }
        else if (a.threeD ^ b.threeD) {
            Log.wtf("dlr", "Class Vektor#distance: Subtract 2D and 3D Vektor. Will return a 2D Vektor!");
        }
        return Math.sqrt(d1+d2);
    }

    public double getLenght() {
        if (threeD) {
            return Math.sqrt(x * x + y * y + z * z);
        } else {
            return Math.sqrt(x * x + y * y);
        }
    }

    /**
     * rotates the x and y coordiante counterclockwise.
     * @param angle in Degree
     */
    public void rotate2D(double angle){
        angle = Math.toRadians(angle);
        x = x * Math.cos(angle) - y * Math.sin(angle);
        y = 2*Math.sin(angle) + 2* Math.sin(angle);
    }

    /**
     * Angle value is between 0 and 180 Degree !
     * @param a Vektor a
     * @param b Vektor b
     * @return Angle between a and b
     */
    public static double getAngleBetween(Vektor a, Vektor b){
        double theta = Math.acos(a.dot(b) / (a.getLenght() * b.getLenght()));
        return Math.toDegrees(theta);
    }

    /**
     * Calculates the dor product
     * @param v Vektor v
     * @return dot product between this and Vektor v
     */
    public double dot(Vektor v){
        if(this.threeD && v.isThreeD()){
            return this.x*v.x + this.y*v.y+this.z * v.z;
        }
        else if (this.threeD ^ v.threeD) {
            Log.wtf("dlr", "Class Vektor#dot: Subtract 2D and 3D Vektor. Will return a 2D Vektor!");
        }
        else{
            return this.x*v.x + this.y*v.y;
        }

        return Double.NaN;
    }

    public void print(String tag) {
        //System.out.println(tag + " X:" + x + " Y:" + y + " Z:" + z + " Lenght:" + this.getLenght());
        Log.i("dlr", tag + " X:" + x + " Y:" + y + " Z:" + z + " Lenght:" + this.getLenght());
    }

    // ****************** GETTER and SETTER ****************** \\
    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getZ() {
        return z;
    }

    public void setZ(double z) {
        this.z = z;
        if(!threeD) threeD = true;
    }

    public boolean isThreeD() {
        return threeD;
    }
}
