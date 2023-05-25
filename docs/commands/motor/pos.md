
# motor pos

Set motor position.

## Usage

```
Usage: robodyno motor pos [OPTIONS] POSITION
```

## Options
* `position` (REQUIRED): 
  * Type: FLOAT 
  * Default: `none`
  * Usage: `position`

  


* `mode`: 
  * Type: Choice(['direct', 'filter', 'track']) 
  * Default: `filter`
  * Usage: `--mode
-m`

  Position control mode. Default is filter mode.


* `bandwidth`: 
  * Type: FLOAT 
  * Default: `10.0`
  * Usage: `--bandwidth
-b`

  Bandwidth of the filter.


* `traj_vel`: 
  * Type: FLOAT 
  * Default: `5.0`
  * Usage: `--traj-vel
-v`

  Trajectory velocity of the position tracking control.


* `traj_acc`: 
  * Type: FLOAT 
  * Default: `10.0`
  * Usage: `--traj-acc
--acc`

  Trajectory acceleration of the position tracking control.


* `traj_dec`: 
  * Type: FLOAT 
  * Default: `10.0`
  * Usage: `--traj-dec
--dec`

  Trajectory deceleration of the position tracking control.


* `vel_ff`: 
  * Type: FLOAT 
  * Default: `0.0`
  * Usage: `--vel-ff`

  Velocity feedforward.


* `torque_ff`: 
  * Type: FLOAT 
  * Default: `0.0`
  * Usage: `--torque-ff`

  Torque feedforward.


* `absolute`: 
  * Type: BOOL 
  * Default: `false`
  * Usage: `--absolute
-a`

  Use absolute position.


* `help`: 
  * Type: BOOL 
  * Default: `false`
  * Usage: `--help`

  Show this message and exit.



## CLI Help

```
Usage: robodyno motor pos [OPTIONS] POSITION

  Set motor position.

Options:
  -m, --mode [direct|filter|track]
                                  Position control mode. Default is filter
                                  mode.
  -b, --bandwidth FLOAT           Bandwidth of the filter.
  -v, --traj-vel FLOAT            Trajectory velocity of the position tracking
                                  control.
  -a, --traj-acc FLOAT            Trajectory acceleration of the position
                                  tracking control.
  -d, --traj-dec FLOAT            Trajectory deceleration of the position
                                  tracking control.
  --vel-ff FLOAT                  Velocity feedforward.
  --torque-ff FLOAT               Torque feedforward.
  -a, --absolute                  Use absolute position.
  --help                          Show this message and exit.
```

