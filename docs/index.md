#  Assignment 4: Cloth Simulation

#### IMPORTANT NOTE: Many of the images are animated `gif`s that run though a part of the simulation to demonstrate some behavior.

## Overview
In this project I implemented a cloth simulation tool and also delved into creating some simple shader programs using GLSL. This project was really interesting and very rewarding. I thought it was really cool to implement a real physical simulator, as it was something I've never done. Creating a relatively simple algorithm that could express the complex behaviour of the cloth was really insightful an cool.

In Part 1 of the project in order to perform cloth simulations, we must implement a cloth model. The simple model that we choose to use in this case is a system of point masses and springs. This involved appropriately constructing a collection of point masses, and spring constraints to accurately model the behavior of the cloth.

In Part 2 of the project, I actually implemented the first parts of the cloth simulation. The simulator accounts for two sources of force, either an external force like gravity, or a spring force exerted on some point mass by one of its constraint springs. This part was really cool, as the code for modeling all of these complex interactions was relatively simple. I used Newton's law to model external forces, and Hooke's Law to model spring forces, and applied all forces to a given point mass using Verlet Integration to update the position of each point mass. I additionally implemented some empirical tools used to ensure that the cloth did not have completely erratic behavior.

In Part 3 & 4 of the project I implemented collision, both with primitives in the scene, like spheres and planes, and with the actual cloth itself. For instance if the cloth were to fold over onto itself it should collide with itself. This involved incorporating spatial hashing to improve the runtime of checking self collision enough so that it could be run in approximately real-time.

In Part 5 I used GLSL shaders to implement a variety of shading and texture mapping methods, such as Blinn Phong shading, texture mapping, bump mapping, displacement mapping, and environment mapping. I found this part interesting as it was rewarding to write concise programs that had such a profound effect on the image. Each of these GLSL shaders dealt with a vertex shader and a fragment shader to determine the output color of a given fragment in the rasterization pipeline.

## Part 1: Masses and Springs

In order to perform cloth simulations, we must implement a cloth model. The simple model that we choose to use in this case is a system of point masses and springs. With some parameters, like a cloth `width` and `height` we can construct a point mass and spring model using a 2 dimensional array of `num_width_points` by `num_height_points` point masses, as well as numerous springs between point masses that act as contraints of our system.

#### `Cloth::buildGrid`

##### Creating `point_masses`

The first task that must be done to implement our cloth simulator is to construct our point mass and spring model. Given the parameters, we want to generate an evenly spaced, `num_width_points` by `num_height_points`  grid that spans `width` and `height`. I compute the desired distance between each point with respect to both the width and the height to satisfy the given parameters:

```cpp
float dist_w = width/(float)(num_width_points-1);
float dist_h = height/(float)(num_height_points-1);
```

There after determining the desired distance between points, I use nested for loops to iterate over the number of desired points in order to create the points.

```cpp
for (int j = 0; j < num_height_points; j++) {
      for (int i = 0; i < num_width_points; i++) {
          ...
          <CREATE POINT MASSES, POPULATE point_masses VECTOR>
          ...
      }
}
```

There are two cases for each point, either the cloth should be oriented as being horizontal, or it should be vertical. If the orientation is `HORIZONTAL`, the `y` coordinate is fixed, while we can compute the `x` and `z` coordinates by using the row and colum of the point mass in the grid according to `i` and `j` in combination with the desired distances between points to find a given point mass' position relative to the origin $(0, 0)$.

```cpp
Vector3D(i*dist_w, 1.0, j*dist_h);
```

If the orientaiton is Vertical, the `x` and `y` coordinates can be computed as above. However, our `z` value is not fixed in this case. Each point mass is slightly offset by some random value between $-1/1000$ and $1/1000$, to add some actual interaction to our system when it is oriented vertically, since the only external force is gravity. We also must then indicate wether or not a point mass is pinned. Each point mass, after being created is inserted into the 1D row-major `point_masses` vector.

##### Creating `springs`

There are three types of springs that I add to the `springs` vector, that exist between the computed point masses in the `point_masses` vector. Each of the springs in the model act as constraints to the system.

**Structural** constraints are springs that go from each point mass to the point mass directly to the left and to the point mass directly above it. These constraints give the cloth its basic structure, and ensure that all of the point masses are connected.

**Shearing** constraints resist shearing. These constraints go from each point mass to the point masses diagonally up to the left and to the right. By having two constraints in both diagonal directions, these constraints also reduce directional bias in our cloth simulation.

**Bending** constraints resist out of plane bending. With only the previous constraints, along a certain row or column, bending the cloth with no resistance with that row/column as the crease faces no resistance. This behaviour doesn't emulate a cloth like we would like. So, we add bending constraints between a point mass and the point mass two away to its left as well as the point mass two above it. These bending constraints should be significantly weaker; we will see this reflected later in the implementation.

Generally, I implement the following for each of the constraints:

```cpp
for (int j = 0; j < num_height_points; j++) {
    for (int i = 0; i < num_width_points; i++) {
        PointMass *pm = &point_masses[j*num_width_points + i];
        if (<constraint can be applied to point mass>) {
            PointMass *pm_1 = <retrieve appropriate point mass for constraint>;
            springs.emplace_back(pm, pm_1, <TYPE>);
        }
        if (<constraint can be applied to point mass>) {
            PointMass *pm_2 = <retrieve appropriate point mass for constraint>;
            springs.emplace_back(pm, pm_2, <TYPE>);
        }
    }
}
```

For each constraint, I go through all of the point masses, and check if a given constraint can be applied to a point mass. This is important for edge cases. For instance, the uppermost, leftmost point mass should have no structural constraints; or the uppermost row of point masses should have no shearing constraints (as the adjacent point masses don't exist). Retrieving the adjacent point mass for each `Spring` constraint is simply using the fact that the array is row-major (i.e. the upper left diagonal of a point mass can be computed using: `point_masses[(j-1)*num_width_points + (i - 1)]`).

#### Results

Below are some screenshots of `scene/pinned2.json` where the point mass and spring model is clear. Each edge is one of the spring constraints

<div align="middle">
        <img src="images/p1/f1.png" align="middle" width="400px"/>
        <figcaption align="middle">scene/pinned2.json from a head on view</figcaption>
      <table style="width=100%">
       <tr>
      <td>
        <img src="images/p1/f2.png" align="middle" width="400px"/>
        <figcaption align="middle">scene/pinned2.json from a different view</figcaption>
      </td>
        <td>
        <img src="images/p1/f3.png" align="middle" width="400px"/>
        <figcaption align="middle">scene/pinned2.json from a third view</figcaption>
      </td>
    </tr>
</table>
</div>

Below is `scene/pinned2.json` with shearing constraints turned off. This is as expected, since without shearing constraints, there are no visual diagonal edges in the grid below, but there are still constraints that go to direct adjacent neighboring point masses:

<div align="middle">
        <img src="images/p1/c_ns.png" align="middle" width="300px"/>
        <figcaption align="middle">scene/pinned2.json without shearing constraints</figcaption>
</div>

Below is the same scene with all constraints other than shearing constraints turned off. Shearing constraints go from a point mass to its upper diagonal point masses, so our grid appears to be oriented diagonally. We could also visuallze this grid as there being an X in each square denoted by all 4 point masses:

<div align="middle">
        <img src="images/p1/c_s.png" align="middle" width="300px"/>
        <figcaption align="middle">scene/pinned2.json with only shearing constraints</figcaption>
</div>

Below are the the results of viewing `scene/pinned2.json` with different springs turned on and off. Below is an image of the scene with all constraints enabled, it becomes essentially the combination of the above two images, since it's putting all of the constraints together:

<div align="middle">
        <img src="images/p1/c_a.png" align="middle" width="300px"/>
        <figcaption align="middle">scene/pinned2.json with all constraints</figcaption>
</div>



## Part 2: Simulation via Numerical Intergration

In this part we implement a mechanism for integrating physics equations for motion in order to apply the forces on our cloth's point masses. Particularly we compute all of the forces on each point mass, and then we use Verlet Integration to compute the new positions for each point mass after having all of the forces applied. Finally, we constrain the position updates from verlet intergration to ensure that the springs and the cloth are not unreasonably deformed each time step, and that the springs only 'stretch' a given amount.

##### Applying Forces

I begin the implementation by setting the `force` vector for each point mass in `point_masses` to be `Vector3D(0,0,0)`. This ensures that we only integrate over forces being applied on the current time step `delta_t`.

In our model, there are two types of forces:

**External Forces**: The external forces are forces that are applied uniformly to all point masses in the cloth. One of these forces is gravity. Each of these external forces can be computed using Newton's Law:


$$
F = ma
$$


The mass for each point mass is computed as follows:

```cpp
double mass = width * height * cp->density / num_width_points / num_height_points;
```

For each accelleration in the vector of `external_accelerations` we can compute the force applied to the point mass $F$, using `mass * external_acceleration`. I add each of these forces to the force vector for each point mass.

**Spring Correction Forces**: We also must account for the spring correction forces for the springs between vertices. Our model consists of the point masses and the spring constraints; we can use Hooke's Law to compute the spring correction force exerted on each point mass in opposing directions:

$$
F_s = k_s * (||p_a - p_b|| - l)
$$


In this case $k_s$ is the spring coefficient, and represents the stiffness of the spring. $l$ is the resting length of the spring. Hooke's Law models the force that pulls the point on the end of a spring from a position back to the resting position. Since there are two points on either end, this force is applied to both point masses in the following manner:

```cpp
if (s.spring_type == <TYPE> && cp->enable_<type>_constraints) {
    double F_s = cp->ks * ((s.pm_a->position - s.pm_b->position).norm() - s.rest_length);
    Vector3D V_b_a = (s.pm_a->position - s.pm_b->position).unit() * F_s;
    Vector3D V_a_b = (s.pm_b->position - s.pm_a->position).unit() * F_s;
    s.pm_a->forces += V_a_b;
    s.pm_b->forces += V_b_a;
}
```

In this example the point mass `pm_a` has a force exerted on it from Hooke's Law, while the opposite force is being exerted on`pm_b`. One could visualize this as the spring force is causing the points to be pulled together or pushed apart, depending on their starting positions. These forces are added to the force vector for each point mass.

One small change is that for bending spring constraints, the spring force is scaled by `0.2`, because we want the bending constraints to be weaker, and have less of an effect on the cloth.

##### Verlet Intergration

Given that we have the sum of all forces applied to each point mass, we can recover the acceleration of each point mass using Newton's Law once again: `pm->forces / mass`. With this acceleration value we use the explicit Verlet Integrator to compute the new position for each point.

Verlet integration computes the position for the next tilmestep, $x_{t + dt}$, as follows:


$$
x_{t + dt} = x_t + v_t dt + a_t dt^2
$$


Where $dt$ = `delta_t`, the length of a single timestep. $a_t$ is the acceleration that we computed above.

With Verlet integration, we approximate the velocity term $v_tdt$ as the change in positions for the last time steps: $x_t - x_{t-dt}$. We also introduce a damping term $d$ to be used to scale down the velocity to emulate the energy lost by friction or heat loss. Thus we get the final expression for updating the position of each point mass:


$$
x_{t + dt} = x_t + (1-d)(x_t - x_{t - dt}) + a_t dt^2
$$


This expression can easily be written in C++ as we have $x_t$ = `position`, $x_{t - dt}$ = `last_position`, and $a_t$ = `pm->forces / mass`. It's important to note that if a point mass is pinned, we want it to remain stationary, and thus its position should not be updated using Verlet Integration.

##### Constraining Position Updates

In order to keep our cloth from behaving too erratically, we want to make sure that it is not unreasonably deformed each time step. To accomplish this, we simply enforce that two point masses connected by a spring can be at most 10% greater than the resting length of the spring apart from each other at the end of a given time step. If this condition is not met, we can apply a correction vector to correct the position of the point masses such that they are not unreasonably deformed.

 We can compute the erroneous distance between the two points on either end of a spring, `s`, as `extra_dist = (s.pm_b->position - s.pm_a->position).norm() - s.rest_length * (1.1);`. Then, the correction vectors applied to each point mass is simply taking each point mass `extra_dist / 2` closer to its neighboring point mass. This ensures that they are always at most `s.rest_length * (1.1)` apart.

#### Results

##### Varying `ks`

`ks` is the spring coefficient used in Hooke's Law and it represents the stiffness of the spring. The higher the value of `ks` the stiffer the spring, and the lower the value the less stiff the spring is.

<div align="middle">
        <img src="images/p2/0.2damp.gif" align="middle" width="400px"/>
        <figcaption align="middle">ks = 5000 (default)</figcaption>
      <table style="width=100%">
       <tr>
      <td>
        <img src="images/p2/ks-low.png" align="middle" width="400px"/>
        <figcaption align="middle">ks = 500, key frame</figcaption>
      </td>
        <td>
        <img src="images/p2/ks-low.gif" align="middle" width="400px"/>
        <figcaption align="middle">ks = 500, animated</figcaption>
      </td>
    </tr>
      <tr>
      <td>
        <img src="images/p2/ks-high.png" align="middle" width="400px"/>
        <figcaption align="middle">ks = 50000, key frame</figcaption>
      </td>
        <td>
        <img src="images/p2/ks-high.gif" align="middle" width="400px"/>
        <figcaption align="middle">ks = 50000, animated</figcaption>
      </td>
    </tr>
</table>
</div>


The biggest visual difference between the two values of `ks` are from the key frames that I've chosen above. To start, a low value of `ks = 500` indicates that the stiffness of the springs are low, so there is a lot more room for bounciness, as the simulation will be more forgiving of erratic motion for point masses, and it will be more forgiving of stretching the spring. This causes the cloth to be less still and sag and bound more as it moves from start to rest. This is extra clear after the cloth has settled to a resting state, as the top edge of the cloth is sagging extra. In this case the force of gravity is allowed to have more of an effect on each point mass' position because the force with which the spring corrections are applied has decreased.

With a high value of `ks = 50000` on the other hand, the spring is now extra stiff, and allows for little mobility for the point masses from the grid. This causes the cloth to be extra stiff as it moves from start to rest. This is clear because there are very few ripples through the cloth. Additionally, the top edge of the cloth does not sag as much, since the stiffness of the springs is greater, and the springs are pulling the point masses to be `rest_length` apart with a greater force.

The value of `ks` changes how 'springy' the cloth is, wether or not it sags or not. Essentially, I though of it as the greater `ks`, the tighter-knit the cloth is, it would take more force to meaningfully bring the point masses apart.

##### Varying `density`

The `density` is used in calculating the mass of a point mass, and is used when computing forces using $F = ma$, and it is also used to recover the acceleration in Verlet Intergration. We can modify the density variable and run multiple simulations:

<div align="middle">
        <img src="images/p2/0.2damp.gif" align="middle" width="400px"/>
        <figcaption align="middle">density = 15 (default)</figcaption>
        <table style="width=100%">
         <tr>
        <td>
          <img src="images/p2/dens-low.png" align="middle" width="400px"/>
          <figcaption align="middle">density = 2, key frame</figcaption>
        </td>
          <td>
          <img src="images/p2/dens-low.gif" align="middle" width="400px"/>
          <figcaption align="middle">density = 2, animated</figcaption>
        </td>
      </tr>
        <tr>
        <td>
          <img src="images/p2/dens-high.png" align="middle" width="400px"/>
          <figcaption align="middle">density = 100, key frame</figcaption>
        </td>
          <td>
          <img src="images/p2/dens-high.gif" align="middle" width="400px"/>
          <figcaption align="middle">density = 100, animated</figcaption>
        </td>
      </tr>
</table>
</div>


The default value of `density` is `15` for reference. When the density is low, the cloth seems to behave like the cloth did when we increased the spring constant `ks`. When we decrease the value of density, two things happen in the way our simulation is run; first, all of the external forces modeled by $F = ma$ become of lower value, since $m$ is decreased; this includes gravity. This effect can be seen because for low values of `density` the model does not sag as much as it did with the default value of density, and it's signifcantly more rigid, as external forces play less of a role on the point masses. Essentially, it's as if the cloth weighed less.

With a high value of `density` the cloth is pulled down substantially, as the force of gravity is increased. Imagine that the cloth becomes heavier. We can see that it sags more on the top edge, as the force of gravity is increased and thus the cloth is pulled down more. There is essentially an inverse relationship between changing `ks` and `density`; as modifying one force changes how the other force is treated.

##### Varying `damping`

The Damping coefficient is used to scale down our velocity term in Verlet Integration to simulate energy loss due to friction, or heat loss. By changing the value of this damping term, we change how much energy is lost each time step. For instance, below is an animated gif of the cloth falling with normal parameters: where `damping = 0.2`:

<div align="middle">
        <img src="images/p2/0.2damp.gif" align="middle" width="400px"/>
        <figcaption align="middle">damping = 0.2 (default)</figcaption>
      <table style="width=100%">
       <tr>
      <td>
        <img src="images/p2/0.08damp.gif" align="middle" width="400px"/>
        <figcaption align="middle">damping = 0.08</figcaption>
      </td>
        <td>
        <img src="images/p2/0.5damp.gif" align="middle" width="400px"/>
        <figcaption align="middle">damping = 0.5</figcaption>
      </td>
    </tr>
</table>
</div>

Notice that changing the damping terms seems to change the amount of time it takes for the cloth to reach a resting state, and for it to slow down. With a damping coefficient of `0.2`  the cloth starts moving downward with a decent amount of energy but slows down considerably when the cloth begins to reach a vertical position, and it assumes a resting state quite quickly.

When we decrease the damping term to `0.08`, so that the energy loss is considerably less than it was previously, we can see that unlike the default simulation, the cloth still has a considerable amount of velocity as it reaches the vertical, and it swings the other way before returning and settling more slowly to a vertical position.

When increasing the damping term to `0.5`, we  can see that the loss of energy and velocity is considerable. It takes much longer for the cloth to reach a resting vertical position, as it begins to slow down on its decent long before the default one did. There are also significantly less ripples in the cloth, as this energy is damped quickly as the cloth decends. Damping has a profound effect on how the cloth settles at a resting state.

#### `pinned4.json`
Below is a screenshot of `pinned4.json` in its final resting state:
<div align="middle">
        <img src="images/p2/pinned4.png" align="middle" width="400px"/>
        <figcaption align="middle">scene/pinned4.json</figcaption>
</div>


## Part 3: Handling Collisions with Other Objects

Currently, the cloth cannot interact with other primitives in the scene, like a sphere or a plane in this case. To impelement collision handling, I must implement each primitive's `collide` method, which checks to see wether or not a point mass is 'inside' the primtive, and if it is, applies a correction vector such that the point is repositioned ot be outside/along the primitive's surface.

For each point mass, I test collision with each primitive in the vector `collision_objects` by calling each `CollisionObject`'s  `collide` method on the point mass.

#### Collisions with Spheres

To test for collision with a sphere, I determine wether or not the distance between the center of the sphere `origin` and the position of the point mass is less than the radius of the sphere (the distance between the center and the sphere's surface). If that is the case, then the point is within the sphere, and a correction vector must be applied.

We want to move the point up to the surface of the sphere, so we first find the point along the surface that is in the same direction from the origin as the point, and then the correction vector is the vector needed to get from the position to the point on the surface of the sphere:

```cpp
Vector3D tangent_point = origin + (pm.position-origin).unit() * radius;
Vector3D correction_vec = tangent_point - pm.last_position;
```

Finally we just apply the correction vector but we also scale it down by friction, $(1 -f)$, making the correction not so exact and striking.

#### Collisions with Planes

To test for collision with a plane, what we really want to test for is wether or not a point mass crosses from one side of the plane to the other. This can be done by analyzing the positions of the point mass' current and last positions, and seeing if the side of the plane that position was on changed between timesteps.

`dot(normal, pm.position - point)` will tell us the distance away from the plane a point is in the direction of that plane's `normal` vector. If the distance was positive on one time step and then negative on another, we know that it must have crossed boundaries during the timestep, and we'd like the cloth to stay only on a single side of the plane. To achieve this we need to apply a correction vector to the point mass' position.

We want to compute where the point mass would have intersected the plane had it kept moving up until the surface. This can be done by projecting the point onto the plane:

```cpp
Vector3D v = pm.position - point;
float dist = dot(normal,v);
Vector3D projected_point = pm.position - dist*normal;
```

I then apply a correction vector that originates at the projected point and is offset slightly by a vector meant to make it so that the point is ever so slightly above the plane:

```cpp
Vector3D offset_tangent = normal.unit() * SURFACE_OFFSET *sgn(dot(normal, pm.last_position - point));
Vector3D correction_vec = (projected_point + offset_tangent) - pm.last_position;
```

Notice that the `sgn` funciton is used to get the sign of the direction that the `offset_tangent` faces away from the plane. This is used to ensure that the cloth is offset in the direction that it intersected the plane from.

Finally we just apply the correction vector but we also scale it down by friction, $(1 -f)$.

#### Results

Below is the result of a normal shaded cloth from `scene/sphere.json` in its final resting state for the default value of `ks = 5000`, and also `ks = 500`, and `ks = 50000`:

<div align="middle">
        <img src="images/p3/spheren.png" align="middle" width="400px"/>
        <figcaption align="middle">scene/sphere.json with ks = 5000</figcaption>
      <table style="width=100%">
       <tr>
      <td>
        <img src="images/p3/spherel.png" align="middle" width="400px"/>
        <figcaption align="middle">scene/sphere.json with ks = 500</figcaption>
      </td>
        <td>
        <img src="images/p3/sphereh.png" align="middle" width="400px"/>
        <figcaption align="middle">scene/sphere.json with ks = 50000</figcaption>
      </td>
    </tr>
</table>
</div>

There is clearly a difference in the final resting state of the cloth for a given value of ks. `ks` represents the stiffness of the springs, it is the constant by which the force of exerted on the point mass by the spring is scaled For the normal vaue of `ks = 5000` the cloth seem to hang down a reasonable amount, yet certainly some folds of the cloth don't hang down quite as much and are sort of scrunched up near the top. For a low value of `ks = 500`, or a low stiffnesss it is clear that the force exerted on the point mass hanging off the sphere is lower. This is because the point masses that are being stopped against gravity by the sphere don't exert as much force on teh point masses draping over the sides of the sphere.

When `ks = 5000`, the stiffness of the springs is much higher, and the force being exerted keeping the point masses together is stronger, this means that the cloth does not spread out as much and can't drape over the sphere like it did for lower values of `ks`. The cloth seems to be more scrunched up near the top in this simulation, while in the other ones, it was more spread out around the top of the sphere. This intuitively makes sense since the value of `ks` indicates the stiffness of the cloth.



Below is the result of plane collision with normal shading, notice that it did not phase through the plane, but the cloth has simply come to rest on its surface:

<div align="middle">
        <img src="images/p3/plane.png" align="middle" width="400px"/>
        <figcaption align="middle">scene/plane.json without plane Collision</figcaption>
</div>

## Part 4: Handling Self Collisions

In Part 3, we added the functionality of allowing our cloth to intersect with other primitives in the scene, but if we were to run a simulation where the cloth is oriented vertically, we will see that the cloth clips into itself, and doesn't behave like a normal cloth would at all.

<div align="middle">
        <img src="images/p4/clip.gif" align="middle" width="400px"/>
        <figcaption align="middle">scene/selfCollision.json without Self Collisions</figcaption>
</div>

This is clearly not the behaviour we'd want. We want the cloth's surface to collide with itself, and to prevent it from clipping like it does above. Particularly, we'd want it to fold on top of itself in this case.

One thing that we could do, is test each point mass to see if it is within some constant of every other point mass, and perform correction vectors if we get two points that are too close together. However, this is very inefficient, particularly $O(n^2)$ where $n$ is the number of point masses. This runtime wouldn't make our simulation nearly real-time anymore

To fix this issue we implement spatial hashing. Spatial hashing partitions the world into 3D box volumes, where each 3D box volume is a bucket in the hash map. Then, rather than looking at all point masses, a given point mass need only look at the point masses that are in the same hash bucket that it hashes to (i.e. they are in the same 3D box volume).

##### `Cloth::hash_position`

We want to make a hash function that finds the 3D box volume that point lies within and the returns an appropriate unique hash value, below is my hash funciton which implements string hashing on the coordinates of the 3D box volume for the point:

```cpp
double w = 3 * width / num_width_points;
double h = 3 * height / num_height_points;
double t = std::max(w, h);
double n_x = floor(pos.x / w) * w;
double n_y = floor(pos.y / h) * h;
double n_z = floor(pos.z / t) * t;

return n_x + (103 * n_y) + (103 * 103 * n_z);
```

##### `Cloth::build_spatial_map`

Using the hash funciton above, the next step is to populate the hash map with each point mass. To implement this part I looped through all of the point masses and got it's `hash_position`. If the `hash_position` was a valid key in the map, I added the point mass the vector of point masses in that 3D box volume, and if not I created a new vector and added the point mass to that new vector.

##### `Cloth::self_collide`

We're given that if a point mass, and a candidate point mass are within $2 * thickness$ distance apart, then we have a self collision and the point mass must have its position corrected to be at least $2 * thickness$ distance away from the candidate point mass. `(pm.position - candidate->position).norm() <= 2 * thickness)`, tells us if we should apply `correction += (pm.position - candidate->position).unit() * (extra_diff)` where `extra_diff` is how much the point should more to be the right distance from the candidate.

For efficiency we only consider point masses that have the same `hash_position` as the point mass we are checking for self collision. Given that there is some correction to be performed for each possible candidate point mass, we average the correction vectors on the point mass. Additionally we scale down the correciton vector average by `simulation_steps`: `(correction / num_corrections) / simulation_steps;`

We take care to not collide a point mass with itself: `!(candidate == &pm)`.

#### Results

Below is the result of running `scene/selfCollision.json` _with_ self collision; I've included 3 screenshots of key moments where the self folding is visible, as well as an animated gif of the whole process:

<div align="middle">


      <table style="width=100%">
       <tr>
      <td>
        <img src="images/p4/sc1.png" align="middle" width="300px"/>
        <figcaption align="middle">scene/selfCollision.json at early self-collision</figcaption>
      </td>
        <td>
        <img src="images/p4/sc2.png" align="middle" width="300px"/>
        <figcaption align="middle">scene/selfCollision.json self-folding</figcaption>
      </td>
           <td>
        <img src="images/p4/sc3.png" align="middle" width="300px"/>
        <figcaption align="middle">scene/selfCollision.json more restful state</figcaption>
      </td>
    </tr>
</table>
​    <img src="images/p4/norm.gif" align="middle" width="400px"/>
​        <figcaption align="middle">scene/selfCollision.json</figcaption>
</div>

We can see that now with self collision implemented, the cloth no longer clips into itself as it falls, instead it folds on top of itself. It essentially crumples up, like a real cloth would! This is because it no longer ignores itself as an object to collide with. Particularly the middle key frame demonstrates this self collision best. It's clear that the cloth has rolled up on top of itself. As the cloth begins to assume a resting state, there is still a portion of the cloth that remains beneath the rest, and there is little to no clipping, so we can see that our self collision is working well.

#### Modifying `density` and `ks`

##### Modifying `ks`

`ks` is the value of the spring constant in Hooke's law. It can be though of as the stiffness of the springs. Let's see what happens when we use a relatively low value vs a relatively high value for this self collision problem. Below are two examples, along with a key frame for two different values of `ks`:

<div align="middle">
      <table style="width=100%">
       <tr>
      <td>
        <img src="images/p4/low-ks.png" align="middle" width="400px"/>
        <figcaption align="middle">ks = 500, key frame</figcaption>
      </td>
        <td>
        <img src="images/p4/low-ks.gif" align="middle" width="400px"/>
        <figcaption align="middle">ks = 500, gif</figcaption>
      </td>
    </tr>
          <tr>
      <td>
        <img src="images/p4/hi-ks.png" align="middle" width="400px"/>
        <figcaption align="middle">ks = 50000, key frame</figcaption>
      </td>
        <td>
        <img src="images/p4/high-ks.gif" align="middle" width="400px"/>
        <figcaption align="middle">ks = 50000, key frame</figcaption>
      </td>
    </tr>
</table>
</div>

The default value for `ks` is `5000` for reference. As stated above `ks` is the spring constant, and thus is the stiffness of the springs. When `ks` is low, our springs are not very stiff. This has the effect of not providing the cloth with much resistance as it accelerates downwards due to gravity. With the spring constant lowered (and the bending constraint lowered signifiacantly), the pointmass' force downwards due to gravity dominates. Notice how the cloth doesn't bend all that much but bunches up very quickly and pushes the folds of the cloth down as the whole cloth falls. With a low `ks` the cloth faces little resistance on the way down.

With a high value of `ks` however, the spring constraints are strengthened and thus the forces exerted on the point masses by the spring constraints is also increased. In this case we can see that the cloth stays relatively together and only folds into a couple larger sized folds when compared to the normal input. This is because, with a high stiffness for the springs, they are more resistant to the cloth being preturbed by other forces. This is why the cloth does not fold as erratically as the normal example, because the stiffness of the cloth's springs exert an extra force keeping its point masses in a relatively correct orientation. The ripples in this cloth are larger and less erratic.

##### Modifying `density`

`density` is the value of the density of each point mass; particularly it is used to compute the mass of the point mass. Let's see what happens when we use a relatively low value vs a relatively high value for this self collision problem. Below are two examples, along with a key frame for two different values of `density`:

<div align="middle">
      <table style="width=100%">
       <tr>
      <td>
        <img src="images/p4/low-dens.png" align="middle" width="400px"/>
        <figcaption align="middle">density = 2, key frame</figcaption>
      </td>
        <td>
        <img src="images/p4/low-density.gif" align="middle" width="400px"/>
        <figcaption align="middle">density = 2, gif</figcaption>
      </td>
    </tr>
          <tr>
      <td>
        <img src="images/p4/high-dens.png" align="middle" width="400px"/>
        <figcaption align="middle">density = 100, key frame</figcaption>
      </td>
        <td>
        <img src="images/p4/high-density.gif" align="middle" width="400px"/>
        <figcaption align="middle">density = 100, key frame</figcaption>
      </td>
    </tr>
</table>
</div>

The default value of `density` is `15` for reference. When the density is low, the cloth seems to behave like the cloth did when we increased the spring constant `ks`. When we decrease the value of density, two things happen in the way our simulation is run; first, all of the external forces modeled by $F = ma$ become of lower value, since $m$ is decreased; this includes gravity. On the otherhand, the acceleration, `forces/mass`, generally increases. This is essentially the same thing that was happening when we increased `ks`. Now gravity isn't as strong of a force, and the forces exerted by the springs is increased, causing the the cloth to act a lot more flowy and not make as many folds as the default case.

When the `density` is increased to be `100`, we experience a similar effect to when we made `ks` lower. Since the mass is increased, the force of gravity is scaled up, while the constaint forces exerted by springs is not. Because of this, one could think about the cloth weighing substantially more. In this case, its heavier, so it falls with less resistence to the spring forces, more in a straight down fassion and it begins to reapidly folw along the base of the cloth.

So changing `density` changes how the cloth folds into itself based on the size and number of the folds. Increasing density causes the folds to be small and numerous, while decreasing it causes the folds to be large and sparse.

## Part 5: Shaders

A shader program is a program that executes certain parts of the rasterization pipeline in parallel on the GPU. This allows for increased performance and speed when shading, especially for real time rendering, as it can be run in parallel. Particularly in this project we worked with two OpenGL shader types, vertex shaders and fragment shaders. A shader is a program that essentially takes input and writes and returns output for use in the graphics pipeline, or for use for another shader program.

##### Vertex Shader

A vertex shader is a shader that takes in vertices and applies geometric transformations to them. For the sake of this project the vertex shader takes in information about a vertex as `in` attributes. It also takes in `uniforms` which are variables that are shared by all instances of the shader. With these shaders, the vertex shader takes in the vertices and converts them to work space for use by the fragment shader. It then writes the values into the `varying` variables, which will later be used by the fragment shader to create lighting and material effects like Bump Mapping.

The vertex shader can also perform more complicated geometric transformations. For instance, in Displacement Mapping we want to actually change the model space positions of the vertices based on the displacement mapping that was passed in, which involved adding some vector in the direction of the normal of a vertex to each vertex.

Finally, the vertex shader pipes values for input into the fragment shader and sets the screen space position of a vertex as well.

##### Fragment Shader

A fragment shader takes in a fragment, something found during the raserization part of the graphics pipeline, as input from the vertex shader. The fragment shader is responsible for determining the output color value for each fragment, or for the sake of this shader really each 'pixel'. This color output is then the color that is displayed for a given fragment/pixel after the gaphics pipeline has finished. With information about the geometry of each fragment from the vertex shader, we can apply shading like lighting effects and material effects as well. Among some of the things we can do are Diffuse Lighting, Blinn Phong Lighting, environment mapping, and texture mapping. Many of these rely on the vertex positions and normals and the positions of lights in the scene and such, many of which are provided by the vertex shader.

In short, the vertex shader takes in vertices and applies geometric transformations, it then feeds these results to both the rest of the graphics pipeline, where it is used by the fragment shader to compute the color of each fragment/pixel, allowing for lighting effects and material modeling.



#### Blinn Phong Shading and Diffuse Shading

In project 3 we looked at very advanced, yet computationally slow and complex shading methods. Blinn Phong shading is a lightweight alternative, that is less computationally intensive and shades a scene by pixel (fragment in this case essentially). However, rather than being based on the physics of light like raytracing was; Blinn Phong shading is based on observation about the types of shading some objects experience.

The general idea is that there are three components to lighting general materials. First, there is ambient light, then there is diffuse lighting, if the surface is coarser, there is a more diffuse lighting effect, versus if its smoother; lastly there is a third component which is specular highlighting.

##### Diffuse Shading

The first shader we implemented was diffuse shading. Diffuse shading essentially indicates that light from a light source is evenly distributed in all directions off of the surface of the object that the light is incident to.

We can model this simply by taking the light intensity that is incident to a given fragment, accounting for light fall off, as well as the position of the light, the position of the vector and the vector normal (most of which are provided by the vertex shader), and we can compute the out color for the fragment when using diffuse lighting.


$$
L_d = k_d (I/r^2)
$$


##### Blinn Phong Shading

As stated above, Blinn Phong is made up of 3 components, the ambient lighting, the diffuse lighting, and the specular highlights.


$$
BlinnPhong = L_a + L_d + L_s
$$


We already derived $L_d$ above, but Blinn Phong also has the ambient and specular parts.

The ambient term for Blinn Phong is just the light that doesn't depend on anything. It is the average of all the light in the scene essentially, and all fragments should emmit that light evenly. So the ambient term is simply $k_a * I_a$ where $k_a$ is the ambient coefficient, and $I_a$ were the ambient light intensity vector.

The specular term for Blinn Phong is reliant of the half vector of the direction to a light source from a vertex and to the camera position from the vertex. Depending on how close the normal vector of a fragment is to the half vector, that determines how0. specular the fragment should be. Below is the specular term of Blinn Phong Shading:


$$
L_s = k_s (I/r^2)max(0, n \cdot h)^p
$$


Increasing $k_s$ increases the specular intensity, while increasing $p$ narrows the reflection lobe, or the size of the specular highlight.

##### Blinn Phong Results

Below are the results of Blinn Phong Shading on `scene/sphere.json` with each component of lighting and then all components. In the below images, $k_a = 0.15, k_d = 1.0, k_s = 0.75, p = 60$, the ambient light is a white light `vec3(1,1,1)`:

<div align="middle">
      <table style="width=100%">
       <tr>
      <td>
        <img src="images/p5/ambient.png" align="middle" width="400px"/>
        <figcaption align="middle">BlinnPhong - ambient component</figcaption>
      </td>
        <td>
        <img src="images/p5/diffuse.png" align="middle" width="400px"/>
        <figcaption align="middle">BlinnPhong - diffuse component</figcaption>
      </td>
    </tr>
          <tr>
      <td>
        <img src="images/p5/specular.png" align="middle" width="400px"/>
        <figcaption align="middle">BlinnPhong - specular component</figcaption>
      </td>
        <td>
        <img src="images/p5/phong.png" align="middle" width="400px"/>
        <figcaption align="middle">BlinnPhong - all components</figcaption>
      </td>
    </tr>
</table>
</div>

The ambient light is clearly a uniform color that is slightly lighter than black, due to the small ambient coefficient. With the diffuse component, the light seems to be diffused by the surface of the cloth on the sphere. With the specular components, its clear to see how the specular highlight translates over to the BlinnPhong image using all of the components. We're using a relatively small (p = 60) specular highlight size, but I think this is a good example, and we can clearly see the areas on the object that are highlighted in the specular compoment only image.

#### Texture Mapping

For a texture mapping shader, we can take advantage of the fact that the `uv` coordinates are passed into the fragment shader, given some 2D sampler texture, we can use `v_uv` to sample from the texture using barycentric coordinates:

```cpp
out_color = texture(u_texture_1, v_uv);
```

##### Result from Texture Mapping

Below I've taken the minecraft diamond blocks and made a texture out of them:

<div align="middle">
        <img src="images/p5/tex.png" align="middle" width="400px"/>
        <figcaption align="middle">scene/sphere.json with texture mapping</figcaption>
</div>



#### Bump and Displacement Mapping Approaches

##### Bump Mapping

To implement Bump Mapping, I changed `Bump.frag` such that it reads in some texture file and changes the normals to present the illusion of there being an actual pattern on the surface of the object. Particularly, each vertex is treated as being in object space, where its normal is pointing directly out of the local vertex. After modifying the normal in object coordinates however, to actually shade the model with modified normal values, we must convert it to model space again, and for that purpose we use a $TBN$ matrix which has the tangent vector as its first column, the cross product of the vertex normal and the tangent vector and then then normal as its last colum.

To get the ammount that the normal should be modified by, we sample the texture map using a single color channel as the value that the normal should be offset. `h` is the function that gets the height from the height map. The normal vector in object space for a vector is then the following:

```cpp
float dU = (h(vec2(v_uv.x + 1.0 / u_texture_2_size.x, v_uv.y)) - h(v_uv)) * u_height_scaling * u_normal_scaling;
float dV = (h(vec2(v_uv.x, v_uv.y + 1.0 / u_texture_2_size.y)) - h(v_uv)) * u_height_scaling * u_normal_scaling;
vec3 no = vec3(-dU, -dV, 1); // object space normal vector
```

This normal vector must then be transformed into model space before it can be passed into Blinn Phong for shading using the bump mapped normals.

##### Displacement Mapping

Displacement Mapping uses the same method above for modifying the normals in the fragment shader. However, we also want the vertex shader to displace the vertices of the object using the value of the height map $h(u, v)$. This can be accomplished by taking the model space position of the vertex, and adding a vector in the direction of its normal vector that is scaled by the value of the height map and some height scaling constant.


$$
p' = p + n *h(u, v) *k_h
$$


The other thing that needs to be changed is that this new updated position needs to be put into `gl_Position` after being converted to screen space: `gl_Position = u_view_projection * v_position;`



After both Bump and Displacement mapping, in the fragment shader, the new normal is plugged into Blinn Phong Shading.

#### Results and Comparisons for Bump and Displacement Mapping

Below are the results from bump mapping when using `texture_3.png`, which is a brick-like texture map.

<div align="middle">
        <img src="images/p5/texture_2.png" align="middle" width="200px"/>
        <figcaption align="middle">texture being mapped</figcaption>
</div>

The normal scaling factor was `100` and the height scaling factor was `0.02`:

<div align="middle">
      <table style="width=100%">
       <tr>
      <td>
        <img src="images/p5/bump_s.png" align="middle" width="400px"/>
        <figcaption align="middle">Bump Mapping on spherewith texture_3.png</figcaption>
      </td>
        <td>
        <img src="images/p5/bump_c.png" align="middle" width="400px"/>
        <figcaption align="middle">Bump Mapping on cloth with texture_3.png</figcaption>
      </td>
    </tr>
</table>
</div>

It's clear to see that the texture does show up on bump mapping, it is particularly clear with the sphere, although you can also see it on the image with the cloth. Bump mapping changes the values of the vertex normals but **only** for the purpose of Blinn Phong shading. This gives us the effect of the surface of the sphere being shaded as if the surface of the sphere was uneven and had a brick patter. However, looking at the sphere we can see that the surface of the sphere is smooth, so the actual positions of the vertices weren't adjusted.

Below are the results now from Displacement mapping when using `texture_3.png`, which is a brick-like texture map. The normal scaling factor was `100` and the height scaling factor was `0.02`:

<div align="middle">
      <table style="width=100%">
       <tr>
      <td>
        <img src="images/p5/disp_s.png" align="middle" width="400px"/>
        <figcaption align="middle">Displacement Mapping on spherewith texture_3.png</figcaption>
      </td>
        <td>
        <img src="images/p5/disp_c.png" align="middle" width="400px"/>
        <figcaption align="middle">Displacement Mapping on cloth with texture_3.png</figcaption>
      </td>
    </tr>
</table>
</div>

It's clear to see that the one of the biggest differences with Displacement Mapping is that it actually changes the positions of the vertices in screen space, which makes the sphere actually look as though it was constructed of multiple bricks. This is one of the advantages to displacement mapping over bump mapping. While both have the same shading effect, as their fragment shaders are essentially the same; displacement mapping actually displaces the vertex positions based on the height value read in by the height map, this is why the surface of the sphere looks very jagged. While it's harder to see in the cloth, it is more irregular when compared to the bump mapped version.

#### Changes in Sphere Mesh's Coarseness

We can observe changes in the resulting render if we change the coarseness of the mesh when using bump and displacement mapping; below are comparisons between different coarseness levels.

<div align="middle">
      <table style="width=100%">
       <tr>
      <td>
        <img src="images/p5/bump_s.png" align="middle" width="350px"/>
        <figcaption align="middle">Bump Mapping with normal coarseness</figcaption>
      </td>
        <td>
        <img src="images/p5/disp_s.png" align="middle" width="350px"/>
        <figcaption align="middle">Displacement Mapping with normal coarseness</figcaption>
      </td>
    </tr>
          <tr>
      <td>
        <img src="images/p5/bump_s_16.png" align="middle" width="350px"/>
        <figcaption align="middle">Bump Mapping with -a/-o 16 coarseness</figcaption>
      </td>
        <td>
        <img src="images/p5/disp_s_16.png" align="middle" width="350px"/>
        <figcaption align="middle">Displacement Mapping with -a/-o 16 coarseness</figcaption>
      </td>
    </tr>
          <tr>
      <td>
        <img src="images/p5/bump_s_128.png" align="middle" width="350px"/>
        <figcaption align="middle">Bump Mapping with -a/-o 128 coarseness</figcaption>
      </td>
        <td>
        <img src="images/p5/disp_s_128.png" align="middle" width="350px"/>
        <figcaption align="middle">Displacement Mapping with -a/-o 128 coarseness</figcaption>
      </td>
    </tr>
</table>
</div>

I personally feel as though the difference is quite negligable for Bump mapping. I think the largest difference between a less coarse and a coarser sphere mesh is the increase smoothness of the edge of the sphere's surface. However, I don't believe coarseness effects the texture beign drawn onto the sphere, regardless of courseness because the texture funciton interpolates within the triangle.

 The **biggest** difference I believe is the quality of the displacement map when varying coarseness. As we can see, when we decrease the coarseness the surface of teh sphere is very lumpy, and doesn't line up quite nicely with the displacement mapping. This is because by decreasing the coarseness, we are decreasing the amount of samples we can take for displacement from the height map, this results in a kind of jagged surface for the sphere. However, when we increase the coarseness of the mesh, these are now more vertices that we can adjust the height of using the height map, so we can see that the surface of the sphere is become idealized, with the divits between bricks being depressed into the sphere and the acutal bricks in the displacement map being eleveated quite evenly.

#### Environment Mapped Reflections

In this final part we approximated the incoming radiance sampel using environment map, which computes the radiance for a given direction. Given that our surface is going to be a mirror surface, for a given camera position, the incoming radiance for a point `v_position` is simply going to be the whatever the specular reflection of the vector from the camera to that point is.

First we get a vector for the direction from the point `v_position` to the camera `u_cam_pos`. Then, I used the `reflect` function to reflect the vector across the vertex's normal to get the sample direction from the environment map which would become the color for the given fragment for `v_position`:

##### Results

Below are the results of environment mapped reflections:

<div align="middle">
      <table style="width=100%">
       <tr>
      <td>
        <img src="images/p5/mirror_s.png" align="middle" width="400px"/>
        <figcaption align="middle">Mirror.frag on sphere</figcaption>
      </td>
        <td>
        <img src="images/p5/mirror_c.png" align="middle" width="400px"/>
        <figcaption align="middle">Mirror.frag on cloth</figcaption>
      </td>
    </tr>
</table>
</div>
