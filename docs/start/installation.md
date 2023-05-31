# 安装

## pip 安装

Robodyno 的软件包可以通过 pip 安装，安装命令如下：

```bash
pip install robodyno
```

## 源码安装

Robodyno 的软件包是完全开源的，你可以在 [GitHub](https://github.com/robodyno/robodyno) 或 [Gitee](https://gitee.com/robodyno/robodyno) 上找到它的源代码。如果你想要对 Robodyno 的软件包进行二次开发，或者想要安装最新的开发版本，可以通过源码安装的方式安装 Robodyno 的软件包。

首先，你需要从 GitHub 或 Gitee 上下载 Robodyno 的源代码，可以通过 `git clone` 命令下载：

```bash
git clone https://github.com/robodyno/robodyno.git
```

或

```bash
git clone https://gitee.com/robodyno/robodyno.git
```

然后，进入 `robodyno` 目录，执行以下命令安装 Robodyno 的软件包：

```bash
cd robodyno
pip install .
```

## 测试安装

安装完成后，你可以通过以下命令测试 Robodyno 的软件包是否安装成功：

```bash
robodyno --version
```

如果安装成功，你应该能够看到 Robodyno 的版本号。
