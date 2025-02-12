# AI Techniques for Robotics

The project combination includes the demonstration of kalman filter, particle filter, searching algorithm, PID control and SLAM algorithm.

## Kalman Filter

In this project, Earth is threatened by a shower of meteorites falling in your location. It is your task to receive
sensor readings of the locations of these meteorites, predict where each of the meteorites will be one tenth of a
second later using Kalman Filters (KFs), and finally, destroy each meteorite before it hits the ground by firing
your laser turret at it.
This project consists of two parts:
- Estimation of the positions of many meteorites given noisy measurements
- Defense: Aim and fire your laser turret at incoming meteorites before they hit the ground


We know the structure of the motion model that governs the motion of the meteorites, but each meteorite
has different coefficients in its equation of motion, which means we can’t just apply the motion model to
predict a particular meteorite’s next location. Kalman Filters allow us to combine our knowledge of the motion
model’s structure and our estimate of our uncertainty of each element in the state of a particular meteorite
with observations of the meteorite’s positions over time to predict where the meteorite will be at a future time.
Since each meteorite has its own motion model coefficients and therefore moves slightly differently than all the
other meteorites, we need one Kalman filter for each meteorite. We’ll want to create and update separate x̄s
and P s for each meteorite, using the Kalman filter equations. The state transition matrix (aka motion model
matrix, F ), measurement model matrix (H), and observation uncertainty matrix (R) are constant and the
same for all meteorites.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/meteorites_localization.gif" title="localization image" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/meteorites_defense.gif" title="defense image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    Estimation of meteorites locations and defense earth.
</div>


## Particle Filter

The goal of this project is to give you practice implementing a particle filter used to localize a
man-made 10.2-meter satellite in a solar system. After completing an intergalactic mission,
it’s time for you to return home. Your satellite is warped through a wormhole and released
into your home solar system in approximate circular orbit around the sun. The satellite
receives measurements of the magnitude of the collective gravitational pull of the planets in
the solar system.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/sat local 2.gif" title="satellite local 2 image" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/sat commu 2.gif" title="satellite commu 2 image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    (Left) Localization of satellite in a solar system; (Right) Localization of satellite in a solar system and communicate with another planet.
</div>


## PID Control

Autonomous drones are used to maintain critical infrastructure, e.g., inspect gas pipelines for leaks. In
this project you will implement a PID controller for an autonomous drone to fly to a target elevation and
horizontal position and hover at some target location for a specified time.
<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/pid_control.gif" title="pid control image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    PID control of drone.
</div>

## Searching Algorithms
In this project, we will implement search algorithms to navigate a robot through a warehouse to pick up and deliver boxes to a designated drop zone area with minimal costs.

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/astar.gif" title="a star image" class="img-fluid rounded z-depth-1" %}
    </div>
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/dp.gif" title="dp image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    A* and dynamic programming algorithms for deterministic process.
</div>

<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/stochastic.gif" title="stochastic image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    Optimal policy finding for stochastic process.
</div>

## SLAM
GraphSLAM operates on a straightforward principle: it deduces a collection of soft constraints from the data, forming a sparse graph. The map and robot path are then determined by resolving these constraints to achieve a globally consistent estimate. In our undertaking, online GraphSLAM is employed to guide the Indiana drones through the forest, searching for treasure while preventing collisions with trees.
<div class="row">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/slam.gif" title="slam image" class="img-fluid rounded z-depth-1" %}
    </div>
</div>
<div class="caption">
    SLAM algorithm navigate the drone to treasure.
</div>
