
# motor vel

Set motor velocity.

## Usage

```
Usage: robodyno motor vel [OPTIONS] VELOCITY
```

## Options
* `velocity` (REQUIRED): 
  * Type: FLOAT 
  * Default: `none`
  * Usage: `velocity`

  


* `mode`: 
  * Type: Choice(['direct', 'ramp']) 
  * Default: `ramp`
  * Usage: `--mode
-m`

  Velocity control mode. Default is ramp mode.


* `ramp`: 
  * Type: FLOAT 
  * Default: `10.0`
  * Usage: `--ramp
-r`

  Ramp rate of the velocity ramp control.


* `torque_ff`: 
  * Type: FLOAT 
  * Default: `0.0`
  * Usage: `--torque-ff`

  Torque feedforward.


* `help`: 
  * Type: BOOL 
  * Default: `false`
  * Usage: `--help`

  Show this message and exit.



## CLI Help

```
Usage: robodyno motor vel [OPTIONS] VELOCITY

  Set motor velocity.

Options:
  -m, --mode [direct|ramp]  Velocity control mode. Default is ramp mode.
  -r, --ramp FLOAT          Ramp rate of the velocity ramp control.
  --torque-ff FLOAT         Torque feedforward.
  --help                    Show this message and exit.
```

