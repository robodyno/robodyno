
# motor init

Initialize motor.

## Usage

```
Usage: robodyno motor init [OPTIONS]
```

## Options
* `pos`: 
  * Type: FLOAT 
  * Default: `0.0`
  * Usage: `--pos
-p`

  Initial position.


* `absolute`: 
  * Type: BOOL 
  * Default: `false`
  * Usage: `--absolute
-a`

  Use absolute position.


* `save`: 
  * Type: BOOL 
  * Default: `true`
  * Usage: `--save`

  Save the absolute initial position.


* `help`: 
  * Type: BOOL 
  * Default: `false`
  * Usage: `--help`

  Show this message and exit.



## CLI Help

```
Usage: robodyno motor init [OPTIONS]

  Initialize motor.

Options:
  -p, --pos FLOAT          Initial position.
  -a, --absolute           Use absolute position.
  --save / -S, --not-save  Save the absolute initial position.
  --help                   Show this message and exit.
```

