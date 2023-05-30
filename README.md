# robodyno

[![pypi](https://img.shields.io/pypi/v/robodyno)](https://pypi.org/project/robodyno/)
[![Python](https://img.shields.io/pypi/pyversions/robodyno)](https://www.python.org/downloads/)
[![wheel](https://img.shields.io/pypi/wheel/robodyno)](https://pypi.org/project/robodyno/#files)
[![download](https://img.shields.io/pypi/dm/robodyno)](https://pypi.org/project/robodyno/)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)

## Installation

```bash
pip install robodyno
```

## Usage

### Command Line Interface

```bash
robodyno --help
```

### Python API

```python
import time
from robodyno.interfaces import CanBus
from robodyno.components import Motor

can_bus = CanBus()
motor = Motor(can_bus)

motor.enable()
motor.set_pos(6.28)
time.sleep(1)
print(motor.get_pos())
motor.disable()
```

For more examples, please refer to `examples` directory.

For more documentation, please refer to [docs](http://101.42.250.169/latest/).
