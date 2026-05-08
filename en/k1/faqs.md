sidebar_position: 10

# FAQ

## Login issue

### What to do if a regular user forgets password?

If a regular user forgets their password, it can be reset using the `root` account. Follow these steps:

1. Boot into the login screen, as shown below:

2. Press **Ctrl + Alt + F3** (make sure the **Fn** key is locked/enabled) to switch to the `tts3` terminal, as shown below:

3. Log in with the username `root` and its password. The default password is `bianbu`, as shown below:

4. Run `export LANG=en_US.UTF-8` to temporarily set the terminal language and avoid character-encoding issues:

5. Run the command `passwd username` to change the user's password, for example, to reset the password for user `bianbu`, as shown below:

6. Press **Ctrl + Alt + F1** to return to the login screen. You can then log in with the new password.

## Update issues

**Update Method**

### Issues During Upgrade on Bianbu 2.0.x

#### Prompt: `Please Install all available updates for your release before upgrading.`

**Cause:** Some installed packages are not consistent with the versions in the current software repository.

**Solution:**

1. Run `sudo apt upgrade` to align installed packages with the repository versions.

2. Re-run the command `do-release-upgrade -f DistUpgradeViewGtk3`.

If the prompt `Please install all available updates for your release before upgrading` still appears, run `apt list --upgradable` to check for available updates. The possible package list and handling methods are as follows:

| Package Name | Handling Method |
| :-- | :-- |
| thunderbird-local-zh-cn | Temporarily uninstall with `sudo apt remove thunderbird-local-zh-cn` |
| thunderbird-local-zh-cn-hans | Temporarily uninstall with `sudo apt remove thunderbird-local-zh-cn-hans` |

## Feedback

If your issue is not resolved, you can report it through one of the following channels:

1. [Submit issues on Gitee](https://gitee.com/bianbu/brdk-doc/issues)
2. [Developer Forum](https://forum.spacemit.com/)
