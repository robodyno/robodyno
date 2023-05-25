
# robodyno

Robodyno command line interface.

## Usage

```
Usage: robodyno [OPTIONS] COMMAND [ARGS]...
```

## Options
* `version`: 
  * Type: BOOL 
  * Default: `false`
  * Usage: `--version`

  Show the version and exit.


* `can_bus`: 
  * Type: STRING 
  * Default: `can0`
  * Usage: `--can-bus
-c`

  CAN bus interface channel.


* `baudrate`: 
  * Type: INT 
  * Default: `1000000`
  * Usage: `--baudrate
-b`

  CAN bus baud rate.


* `help`: 
  * Type: BOOL 
  * Default: `false`
  * Usage: `--help`

  Show this message and exit.



## CLI Help

```
Usage: robodyno [OPTIONS] COMMAND [ARGS]...

  Robodyno command line interface.

Options:
  --version               Show the version and exit.
  -c, --can-bus TEXT      CAN bus interface channel.
  -b, --baudrate INTEGER  CAN bus baud rate.
  --help                  Show this message and exit.

Commands:
  list     List all devices on the CAN bus.
  monitor  Monitor all devices on the CAN bus.
  motor    Motor commands.
```

