---
sidebar_position: 9
slug: /07_FAQ
---

# 7.FAQ

## 登录

### 普通用户忘记密码怎么解决？

普通用户忘记密码，可以通过 root 用户修改。以下是具体步骤。

1. 开机进入登录界面，如下图


2. 按下键盘 **Ctrl + Alt + F3** 组合键（注意要先 lock Fn），进入 tty3 终端，如下图


3. 输入用户名 `root` 和密码，默认密码是 `bianbu` ，如下图


4. 运行 `export LANG=en_US.UTF-8`，临时修改终端语言，避免乱码

5. 运行 `passwd 用户名` 修改该用户密码，例如用户`bianbu`，如下图


6. 按下键盘 **Ctrl + Alt + F1** 组合键，切回登录界面，使用新密码登录即可。

## 更新


**更新方法**


### Bianbu 2.0.x 升级时可能遇到的问题

#### 提示 `请在升级前安装您的发行版所有可用更新`

**原因：** 系统当前安装的部分软件包版本和软件源里的版本不一致。

解决：

1. 执行 `sudo apt upgrade`，重新安装回源里的版本。

2. 重新执行 `do-release-upgrade -f DistUpgradeViewGtk3` 即可。

如仍然提示 `请在升级前安装您的发行版所有可用更新`。可执行 `apt list --upgradable` 查看可用更新，可能的包列表和处理方式如下，

| 包名                         | 处理方式                                               |
| :--------------------------- | :----------------------------------------------------- |
| thunderbird-local-zh-cn      | 暂时卸载`sudo apt remove thunderbird-local-zh-cn`     |
| thunderbird-local-zh-cn-hans | 暂时卸载`sudo apt remove thunderbird-local-zh-cn-hans` |

## 问题反馈

如仍有问题，可通过以下渠道反馈:

1. [gitee提交issues](https://gitee.com/bianbu/brdk-doc/issues)
2. [开发者论坛](https://forum.spacemit.com/)