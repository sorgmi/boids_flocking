package boids;

import java.util.ArrayList;
import Vektor;

/**
 * Implements Flocking behaviour with the following rules:<br /> 1. Seek to the Leader-position, if one is selected
 * <br />2. separate im distance smaller than {@link BoidsFlockFollowLeader#SEPARATION_RADIUS}
 * <br />3. cohesion, but only if no leader is selected {@link BoidsFlockFollowLeader#cohesion(IAgent)}
 * <br />4. Collison Avoidance, with the nearest obstacle {@link BoidsFlockFollowLeader#avoidObstacles(IAgent)}
 *
 * @see <a href="http://gamedevelopment.tutsplus.com/tutorials/understanding-steering-behaviors-leader-following--gamedev-10810">http://gamedevelopment.tutsplus.com/tutorials/understanding-steering-behaviors-leader-following--gamedev-10810</a>
 */

public class BoidsFlockFollowLeader {

    // Parametes
    public double MAX_SPEED = 1.5f;
    public double MAX_FORCE = 0.9f;
    public double SafetyBoxDistance = 2.0;
    public double SEPARATION_RADIUS = 7.0f;
    public double MIN_Z_TO_FOLLOW_LEADER = -1.0f;
    public double NEIGHBOUR_RADIUS = 4.0f;
    public double OBSTACLE_PERCEPTION = 4.5f;
    public double SLOWING_RADIUS = 6.0f;

    private float AGENT_RADIUS = 0.25f;

    private float HIGH_VALUE = 1000;  // Initial value for finding the minimum distance between two objects
    private ArrayList<IAgent> agents = new ArrayList<>();
    private ArrayList<Obstacle> obstacles = new ArrayList<>();
    private IAgent leader;


    public BoidsFlockFollowLeader(ArrayList<IAgent> list, ArrayList<Obstacle> obstacles) {
        this.agents = list;
        this.obstacles = obstacles;
    }

    /**
     * Apply all rules to each agent (boid), update his position and send it.
     * At the end the accerlaration vector of each agent will be set to null
     */
    public void iterate() {
        leader = null;
        for (IAgent q : agents) {
            if (q.isSelected()) leader = q;
        }
        for (IAgent boid : agents) {

            Vektor acceleration = boid.getAccelaration();
            Vektor velocity = boid.getVelocity();

            Vektor seek = seekLeader(boid, leader);
            Vektor separate = separate(boid);
            Vektor cohesion = cohesion(boid);
            Vektor obstacle = avoidObstacles(boid);
            Vektor alignment = avoidObstacles(boid);

            acceleration.add(seek);
            acceleration.add(separate);
            acceleration.add(cohesion);
            acceleration.add(obstacle);
          //  acceleration.add(alignment);

            velocity.add(acceleration);
            velocity.limit(MAX_SPEED);
            Vektor pos = boid.getPosition();
            pos.add(velocity);
            safetyBox(boid,pos);

            acceleration.multiply(0);
        }

        analyzeAvgLeader();
        analyzeMin();
        analyzeMinDistObst();
    }


    private Vektor alignment(IAgent boid){
        Vektor force = new Vektor(0,0,0);
        int count = 0;
        for (IAgent neighbour : agents) {
            double distance = Vektor.subtract(boid.getPosition(), neighbour.getPosition()).getLenght();
            if (distance < NEIGHBOUR_RADIUS && neighbour != boid) {
                force.add(neighbour.getVelocity());
                count++;
            }
        }
        if(count > 0){
            force.multiply(1/count);
            force.normalize();
            force.multiply(MAX_SPEED);
            Vektor seek = Vektor.subtract(boid.getVelocity(), force);
            seek.limit(MAX_FORCE);
            return  seek;
        }
        else{
            return force;
        }
    }


    /**
     * Only the nearest obstacle is avoided and only if the predicted smallest distance is below 1.0m
     * then a force pointing away from the obstacle with 1.75*{@link BoidsFlockFollowLeader#MAX_FORCE} is returend
     * @param boid current boid
     * @return force pointing away from the obstacels center
     */
    private Vektor avoidObstacles(IAgent boid) {
        Vektor force = new Vektor(0, 0, 0);
        double closestDistance = HIGH_VALUE;
        Obstacle obstacle = null;
        // Find the nearest obstacle
        for (Obstacle o : obstacles) {
            double d = Vektor.subtract(o.location, boid.getPosition()).getLenght();
            d = d - o.radius - AGENT_RADIUS;
            if (d < closestDistance && d < OBSTACLE_PERCEPTION) {
                obstacle = o;
                closestDistance = d;
            }
        }
        // Avoid nearest obstacle
        if (obstacle != null && closestDistance != HIGH_VALUE) {
            Vektor direction = new Vektor(boid.getVelocity());
            direction.multiply(100);
            direction.normalize();
            for(float i=0; i < (OBSTACLE_PERCEPTION*100); i += 10){
                Vektor v = new Vektor(direction);
                v.multiply(i);
                v.multiply(0.01);
                v.add(boid.getPosition());

                Vektor f = Vektor.subtract(obstacle.location, v);
                double distance = f.getLenght();
                distance = distance - obstacle.radius - AGENT_RADIUS *2;

                if(distance < 1.0f){
                    f.normalize();
                    f.multiply(MAX_FORCE * 1.75); // 1/distance
                    f.limit(MAX_FORCE * 1.75);
                    return f;
                }
            }
        }

        return force;
    }


    /**
     * Creates a force towards a goal.<br /> If the distance between the boid and the goal is below {@link BoidsFlockFollowLeader#SLOWING_RADIUS}
     * then a damping force is created. Steering force = desired velocity - current velocity
     * Damping force: wenn alle Agenten nahe dem Ziel, dannn Geschwindigkeiten verringern, damit der Schwarm nicht
     * so stark oszilliert
     *
     * @param boid Current boid
     * @return Vektor computed force
     */
    private Vektor seekLeader(IAgent boid, IAgent leader) {
        if (leader != null) {
            Vektor force = Vektor.subtract(boid.getPosition(), leader.getPosition());

            double distance = force.getLenght();
            double dampingFactor = 1;
            if (distance < SLOWING_RADIUS) {
                dampingFactor = distance / SLOWING_RADIUS;
            }

            force.normalize();
            force.multiply(MAX_SPEED);
            force.multiply(dampingFactor);
            force.sub(boid.getVelocity()); // Steering = desiredvelocity - current velocity
            force.limit(MAX_FORCE);
            return force;
        }

        return new Vektor(0, 0, 0);
    }

    private Vektor seekGoal(IAgent boid, Vektor goal) {
        Vektor force = Vektor.subtract(boid.getPosition(), goal);

        double distance = force.getLenght();
        double dampingFactor = 1;
        if (distance < SLOWING_RADIUS) {
            dampingFactor = distance / SLOWING_RADIUS;
        }

        force.normalize();
        force.multiply(MAX_SPEED);
        force.multiply(dampingFactor);
        force.sub(boid.getVelocity()); // Steering = desiredvelocity - current velocity
        force.limit(MAX_FORCE);
        return force;

    }

    /**
     * Cohesion force: Here only active, if a leader is selected
     *
     * @param boid current agen
     * @return a Vektor pointing to the average position of all neighbours
     */
    private Vektor cohesion(IAgent boid) {
        if (leader == null) {
            Vektor force = new Vektor(0, 0, 0);
            int count = 0;
            for (IAgent neighbour : agents) {
                double distance = Vektor.subtract(boid.getPosition(), neighbour.getPosition()).getLenght();
                if (distance < NEIGHBOUR_RADIUS && neighbour != boid) {
                    force.add(neighbour.getPosition());
                    count++;
                }
            }

            if (count > 0) {
                force.multiply(1 / count);
                Vektor v = Vektor.subtract(boid.getPosition(), force);
                v.limit(MAX_FORCE);
                //return v;
                return seekGoal(boid, v); // maybe a bit better (more constant)
            }
            return force;
        }

        return new Vektor(0, 0, 0);
    }

    /**
     * Calculates the seperate force (Average vetors pointing away from the neighbour. Weight (1/r) is not applied
     * --> better results
     *
     * @param boid current boid
     * @return Vektor with separate Force
     */
    private Vektor separate(IAgent boid) {
        int neighbourCount = 0;
        Vektor force = new Vektor(0, 0, 0);

        for (IAgent q : agents) {
            double distance = Vektor.subtract(boid.getPosition(), q.getPosition()).getLenght();
            if (distance < SEPARATION_RADIUS && q != boid) {
                Vektor sep = new Vektor(boid.getPosition());
                sep.sub(q.getPosition());
                sep.normalize();
                // sep.multiply(1/distance);
                force.add(sep);
                neighbourCount++;
            }
        }

        if (neighbourCount > 0) {
            force.sub(boid.getVelocity()); // Here, below If or not all - What's the best?
        }

        return force;
    }

    private void analyzeMinDistObst() {
        double min = HIGH_VALUE;
        for (Obstacle o : obstacles) {
            for (IAgent q : agents) {
                if (!q.isSelected()) {
                    double d = Vektor.subtract(o.location, q.getPosition()).getLenght();
                    d = d - o.radius - AGENT_RADIUS;
                    if (d < min) {
                        min = d;
                    }
                }
            }
        }
        if (min != HIGH_VALUE) {
            floatPublisherminDistObstacle.sendValue(min);
        }
    }

    private void analyzeAvgLeader() {
        if (leader != null) {
            int count = 0;
            double d = 0;
            for (IAgent q : agents) {
                if (q != leader) {
                    Vektor v = Vektor.subtract(q.getPosition(), leader.getPosition());
                    d += v.getLenght();
                    d -= 2*AGENT_RADIUS;            // beachte Quad Radius / Breite
                    count++;
                }
            }

            if (count > 0) {
                d = d / count;
                floatPublisherAvgLeader.sendValue(d);
            }
        }

    }

    private void analyzeMin() {
        double d = HIGH_VALUE;
        double h = HIGH_VALUE;
        for (IAgent q : agents) {
            for (IAgent other : agents) {
                if (q != other) {
                    Vektor v = Vektor.subtract(q.getPosition(), other.getPosition());
                    double dist = v.getLenght();
                    dist -= 2*AGENT_RADIUS;         // beachte Quad Radius / Breite
                    if (dist < d) {
                        d = dist;
                    }
                }
            }
            if (q.getPosition().getZ() < h) h = q.getPosition().getZ();
        }

        if (d != HIGH_VALUE) {
            //System.out.println(d);
            floatPublisherminDist.sendValue(d);
        }
        if (h != HIGH_VALUE) {
            floatPublisherminZ.sendValue(h);
        }
    }

    private boolean canBoidSeeNeighbour(IAgent boid, IAgent neighbour){
        Vektor velocity = new Vektor(boid.getVelocity());
        //velocity1.normalize();
        //velocity1.multiply(NEIGHBOUR_RADIUS);
        Vektor ab = Vektor.subtract(boid.getPosition(), neighbour.getPosition());

        int angle = (int) Vektor.getAngleBetween(velocity, ab);

        if( (180-angle) < 123 && ab.getLenght() <= NEIGHBOUR_RADIUS){
            System.out.println("see you");
            return true;
        }

        return false;
    }


}
