### Model Predictive Control Project

#### Results

#### Model
The model I used was the Global Kinematic Model found in lesson 19. This
model ignores certain things like tire forces, gravity, and mass.

The model is based on:
x = x coordinate
y = y coordinate
psi = orientation
v = velocity
cte = cross track error
epsi = orientation error

The model outputs a steering and throttle actuator value. That moves
that vehicle close to the polynomial fitted line we derive from waypoints
that represent the center of the road.

It does this by trying to predict the vehicles motion N steps into the
future based on the current state, using the update equations:

```
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

We provide some constraints and cost functions that the solver tries to
optimize for to produce good values that match the center line.

We add some weights to give hints to the solver for what constraints we
care more about. For example cte_weight and epsi_weight were set to 2000
because we highly value how far we are from the center and that our
orientation is not drastically different. This is in comparison to no
additional weight applied to velocity state which allows the solver to
slow down or speed up to stay close to the center. Additionally
delta_weight and delta_dt_weight were set relatively high at 2000 as
well to make sure the steering angle was correct and that the change
between them was small. The later being an attempt to minimize
jerkiness.

Additionally some constraints placed on the solver involve simulator
max/mins. For example the vehicle can't turn greater than 25 deg in
either direction and the accelerator can only go from -1 to 1.

### N and dt
As stated above when MPC tries to fit the center line it does so by
projecting where it thinks the car will be N steps into the future at dt
intervals. The values selected were done experimentally, since too large
an N * dt would be computationally expensive and be inaccurate as
the solver has too much leeway to optimize for the future. Too small and
we could have a lot of jerkiness optimizing for only what is directly in
front.

I started with the values from the MPC quiz of N=25 and t=0.05 and that
was way too much. It produced some crazy trajectories going into turns
and even on straight sections the car swayed a lot.

I then moved to values provided by the Q&A youtube video of N=10 and
dt=.1 which is approximately 1 sec into the future. This seemed to work
well with the choosen desired speed of 60.

### Polynomial Fit
Polynomial fit is a way of generating the coefficients for some
polynomial of a desired order that will be close to a set of points
given.

### Latency
In the real world it actuators do not immediately take effect. This is
modeled in the code with a 100ms sleep. In order to compensate for this
delay when we set the initial state we use the model described above to
move the car forward 100ms in time. This is helped by the fact that when
we translated the position of the vehicle from map coordinates to the
vehicle's frame of reference we are able to drop out a lot of terms
because a lot of terms are zero in the vehicle's reference point.



