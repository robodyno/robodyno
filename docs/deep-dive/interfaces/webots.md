# Webots

[Webots](https://cyberbotics.com/) 是一个开源的机器人仿真环境，它可以模拟机器人的运动学和动力学，同时还可以模拟机器人的传感器。Robodyno 当前为 Webots 提供了伺服减速电机和直线运动模组的仿真模型，你可以在 Webots 中测试你的代码，而不需要实际的硬件模组。

## 安装

### Webots

Webots 的安装请参考 [Webots 官方文档](https://cyberbotics.com/doc/guide/installation-procedure)。为了能够使用 Robodyno 的仿真模型，你需要安装 Webots R2023a 或者更新的版本。

### Robodyno 仿真模型

Robodyno 仿真模型可以在 [GitHub](https://github.com/robodyno/robodyno_webots_models) 或 [Gitee](https://gitee.com/robodyno/robodyno_webots_models) 上下载。

下载完成后将 `robodyno_webots_models` 文件夹中的 `protos` 放入自己的 Webots 项目文件夹中，即可使用 Robodyno 的仿真模型。同时你也可以在 `robodyno_webots_models` 的 `worlds` 文件夹中找到一些使用 Robodyno 仿真模型的 Webots 项目示例。

## 使用

Robodyno 对 Webots 的 Python API 进行了封装，你可以直接使用 Robodyno 的 API 来控制 Webots 中的模型。

Robodyno 提供了一个 `Webots` 类，你可以通过创建一个 `Webots` 类的接口实例来使用 Robodyno 的 API。

```python
from robodyno.interfaces import Webots
from robodyno.components import Motor

webots = Webots()
motor = Motor(webots)
```

`webots` 对象会自动连接到 Webots 控制器绑定的机器人，同时会创建一个后台线程来持续调用 Webots 的 `step` 函数以更新机器人的状态。

如果你不希望仿真环境在 `webots` 对象创建后立即开始更新，你可以在创建 `webots` 对象时传入 `start=False` 参数，然后在你需要仿真环境开始运行时调用 `webots.start()`。

```python
from robodyno.interfaces import Webots

webots = Webots(start=False)
# do something
webots.start()
```

`webots` 对象默认会获取当前机器人的 `BasicTimeStep` 作为仿真环境的时间步长，如果你想要修改时间步长，你可以在创建 `webots` 对象时传入 `time_step` 参数。

```python
from robodyno.interfaces import Webots

webots = Webots(time_step=32)
```

如果你想要在仿真环境每步更新时执行一些操作，你可以在创建 `webots` 对象时传入 `step_callback` 参数，`step_callback` 参数应该是一个函数，该函数会在仿真环境每步更新时被调用。

```python
from robodyno.interfaces import Webots

def step_callback(webots):
    # do something
    # e.g. print(webots.time())
    pass

webots = Webots(step_callback=step_callback)
```

`webots` 对象提供了一些方法来管理仿真环境的时间，你可以通过 `webots.time()` 获取仿真环境的当前时间，通过 `webots.sleep(seconds)` 让仿真环境暂停 `seconds` 秒。

```python
from robodyno.interfaces import Webots

webots = Webots()
print(webots.time())
webots.sleep(1)
print(webots.time())
```

## API

详细的 API 请查看 [Webots](../../../references/interfaces/webots)。
