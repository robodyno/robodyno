
# motor config

Configure motor parameters.

## Usage

```
Usage: robodyno motor config [OPTIONS]
```

## Options
* `vel_limit`: 
  * Type: BOOL 
  * Default: `false`
  * Usage: `--vel-limit
-v`

  Set velocity limit.


* `current_limit`: 
  * Type: BOOL 
  * Default: `false`
  * Usage: `--current-limit
-c`

  Set current limit.


* `pid`: 
  * Type: BOOL 
  * Default: `false`
  * Usage: `--pid
-p`

  Set PID gains.


* `heartbeat_rate`: 
  * Type: BOOL 
  * Default: `false`
  * Usage: `--heartbeat-rate
-h`

  Set heartbeat rate.


* `new_id`: 
  * Type: BOOL 
  * Default: `false`
  * Usage: `--new-id
-i`

  Set new ID.


* `can_baudrate`: 
  * Type: BOOL 
  * Default: `false`
  * Usage: `--can-baudrate
-b`

  Set CAN baudrate.


* `help`: 
  * Type: BOOL 
  * Default: `false`
  * Usage: `--help`

  Show this message and exit.



## CLI Help

```
Usage: robodyno motor config [OPTIONS]

  Configure motor parameters.

Options:
  -v, --vel-limit       Set velocity limit.
  -c, --current-limit   Set current limit.
  -p, --pid             Set PID gains.
  -h, --heartbeat-rate  Set heartbeat rate.
  -i, --new-id          Set new ID.
  -b, --can-baudrate    Set CAN baudrate.
  --help                Show this message and exit.
```

