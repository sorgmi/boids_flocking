package boids;

import Vektor;


public interface IAgent {
    Vektor getPosition();

    Vektor getVelocity();

    Vektor getAccelaration();

    void sendWaypoint(float x, float y, float z, float yaw, float vmaxXY, float vmaxZ);

    boolean isSelected();


}
