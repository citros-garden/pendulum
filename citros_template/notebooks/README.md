## Systems

Two simulation scenario are belong to the current project: `simulation_double_pendulum` which simulates the motion of the double pendulum, and `simulation_system_with_spring` - the double pendulum in which the second pendulum is connected with the additional third pendulum by spring:

![Schema](systems_schema.png)

## Double Pendulum

For notebook `double_pendulum.ipynb` we created two batches using `simulation_double_pendulum` simulation scenario:
- `double_pendulum_angles` - we varied the initial angles of the first pendulum by selecting values randomly from a normal distribution with a mean of 120 degrees and a standard deviation of 5 degrees;
- `double_pendulum_vel` - we varied the initial velocity of the second pendulum, choosing values randomly from a normal distribution with a mean of 90 degrees per second and a standard deviation of 5 degrees per second.

The initial parameters for these simulations are presented in [Initial Parameters](#initial-parameters) section, the output format is listed in [Output Format](#output-format) section.

### System Parameters

parameter|description
--|--
l1 | Length of the first pendulum, m
l2 | Length of the second pendulum, m
m1 | Mass of the first pendulum, kg
m2 | Mass of the second pendulum, kg
a1_0| Initial angle of the first pendulum, counted counterclockwise, degrees
a2_0| Initial angle of the second pendulum, counted counterclockwise, degrees
v1_0| Initial angular velocity of the first pendulum, counted counterclockwise, degrees per second
v2_0| Initial angular velocity of the second pendulum, counted counterclockwise, degrees per second
T | Time of the simulation, seconds
h | Step of the simulation, seconds

### Initial Parameters

- Batch `double_pendulum_angles`:

```js
...
"double_pendulum": {
    "double_pendulum": {
        "ros__parameters": {
            "publish_freq": 10.0,
            "l1": 1.2,
            "l2": 1.0,
            "m1": 1.0,
            "m2": 1.0,
            "a1_0": {
                "function": "numpy.random.normal",
                "args": [120.0, 5.0]
            },
            "a2_0": -30.0,
            "v1_0": 0.0,
            "v2_0": 0.0,
            "T": 10.0,
            "h": 0.01
        }
    }
}
...
```

- Batch `double_pendulum_vel`:
```js
...
"double_pendulum": {
    "double_pendulum": {
        "ros__parameters": {
            "publish_freq": 10.0,
            "l1": 1.2,
            "l2": 1.0,
            "m1": 1.0,
            "m2": 1.0,
            "a1_0": 120.0,
            "a2_0": -30.0,
            "v1_0": 0.0,
            "v2_0": {
                "function": "numpy.random.normal",
                "args": [90, 5]
            },
            "T": 5.0,
            "h": 0.01
        }
    }
}
...
```

### Output Format

### Output Format

```js
{
    't': 'float',
    'p1': {
        'x': 'float',
        'y': 'float'
    },
    'p2': {
        'x': 'float',
        'y': 'float'
    }
}
```

parameter | description
--|--
t | time, s
p1.x|x coordinate of the first pendulum, m
p1.y|y coordinate of the first pendulum, m
p2.x|x coordinate of the second pendulum, m
p2.y|y coordinate of the second pendulum, m

## System with Spring

For `system_with_spring.ipynb` notebook we created `spring_system_angles` batch by `simulation_system_with_spring` simulation scenario. In the [initial parameters](#initial-parameters-1) the initial angles of the second pendulum were randomly selected from a normal distribution with a mean of 10 degrees and a standard deviation of 5 degrees.

### System Parameters

parameter|description
--|--
l1 | Length of the first pendulum, m
l2 | Length of the second pendulum, m
l3 | Length of the third pendulum, m
m1 | Mass of the first pendulum, kg
m2 | Mass of the second pendulum, kg
m3 | Mass of the second pendulum, kg
a1_0| Initial angle of the first pendulum, counted counterclockwise, degrees
a2_0| Initial angle of the second pendulum, counted counterclockwise, degrees
a2_0| Initial angle of the third pendulum, counted counterclockwise, degrees
v1_0| Initial angular velocity of the first pendulum, counted counterclockwise, degrees per second
v2_0| Initial angular velocity of the second pendulum, counted counterclockwise, degrees per second
v3_0| Initial angular velocity of the third pendulum, counted counterclockwise, degrees per second
T | Time of the simulation, seconds
h | Step of the simulation, seconds
x0 | Horizontal distance between attachment points of the first and third pendulums, m 
k | Spring constant, kg/s^2
l0 | Unstretched spring length, m

### Initial Parameters

- Batch `spring_system_angles`:
```js
...
"system_with_spring": {
    "system_with_spring": {
        "ros__parameters": {
            "publish_freq": 10.0,
            "l1": 0.08,
            "l2": 0.2,
            "l3": 0.32,
            "lk": 0.2,
            "m1": 5.0,
            "m2": 2.0,
            "m3": 3.0,
            "a1_0": 30.0,
            "a2_0": {
                "function": "numpy.random.normal",
                "args": [10.0, 5.0]
            },
            "a3_0": -30.0,
            "v1_0": 0.0,
            "v2_0": 0.0,
            "v3_0": 0.0,
            "x0": 0.1,
            "k": 100.0,
            "l0": 0.05,
            "T": 10.0,
            "h": 0.05
        }
    }
}
...
```

### Output Format

```js
{
    't': 'float',
    'p1': {
        'x': 'float',
        'y': 'float'
    },
    'p2': {
        'x': 'float',
        'y': 'float'
    },
    'p3': {
        'x': 'float',
        'y': 'float'
    },
    'spr': {
        'x0': 'float',
        'x1': 'float',
        'y0': 'float',
        'y1': 'float'
 }
}
```

parameter | description
--|--
t | time, s
p1.x|x coordinate of the first pendulum, m
p1.y|y coordinate of the first pendulum, m
p2.x|x coordinate of the second pendulum, m
p2.y|y coordinate of the second pendulum, m
p3.x|x coordinate of the third pendulum, m
p3.y|y coordinate of the third pendulum, m
spr.x0|x coordinate of the spring attachment to the third pendulum, m
spr.x1|x coordinate of the spring attachment to the second pendulum, m
spr.y0|y coordinate of the spring attachment to the third pendulum, m
spr.y1|x coordinate of the spring attachment to the second pendulum, m