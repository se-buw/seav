## Issue-1

```
user@DESKTOP-1NU5M8F:~/ros2_ws$ colcon build --symlink-install
Starting >>> my_robot_controller
--- stderr: my_robot_controller
/usr/lib/python3/dist-packages/setuptools/command/easy_install.py:158: EasyInstallDeprecationWarning: easy_install command is deprecated. Use build and pip and other standards-based tools.
  warnings.warn(
---
Finished <<< my_robot_controller [1.48s]

Summary: 1 package finished [1.83s]
  1 package had stderr output: my_robot_controller
```

One probable solution is to downgrade the version of setuptools to maybe something and then run your previous command again.

``` bash
user@DESKTOP-1NU5M8F:~/ros2_ws/src/my_robot_controller/my_robot_controller$ pip3 list | grep setuptool
s
setuptools                           59.6.0
user@DESKTOP-1NU5M8F:~/ros2_ws/src/my_robot_controller/my_robot_controller$ pip3 install setuptools==5
8.2.0
Defaulting to user installation because normal site-packages is not writeable
Collecting setuptools==58.2.0
  Downloading setuptools-58.2.0-py3-none-any.whl (946 kB)
     ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 946.1/946.1 KB 6.6 MB/s eta 0:00:00
Installing collected packages: setuptools
Successfully installed setuptools-58.2.0
user@DESKTOP-1NU5M8F:~/ros2_ws/src/my_robot_controller/my_robot_controller$
```

## Query-1
Why we are every time sourcing the bash?
```bash
user@DESKTOP-1NU5M8F:~/ros2_ws$ source ~/.bashrc
```

Sure! Let's break it down in simpler terms.

### What Does `source` Do?

The `source` command in Bash is used to run commands from a file in your current terminal session. This means any changes made in that file will take effect right away without needing to start a new terminal window.

### Why Do We Use `source ~/.bashrc`?

1. **Apply Changes**: When you make changes to your `.bashrc` file (which is a configuration file for your terminal), you want those changes to take effect immediately. By running `source ~/.bashrc`, you tell your terminal to read the file and apply any new settings or commands you've added.

2. **Keep Everything in One Place**: The `.bashrc` file can store useful things like:
   - **Aliases**: Shortcuts for commands (e.g., making `ll` run `ls -l`).
   - **Environment Variables**: Settings that help programs know where to find files or how to behave.

### How It Works

- **Current Session**: When you use `source`, everything happens in the terminal you're currently using. If you run a script normally, it would run in a separate environment, and any changes wouldn’t affect your current session.
  
- **No New Window Needed**: You don’t have to close and reopen your terminal to see the changes; just run the `source` command, and everything updates right away.

### Example

If you added a new alias to your `.bashrc` like this:

```bash
alias gs='git status'
```

After saving that change, you would run:

```bash
source ~/.bashrc
```

Now, you can use the shortcut `gs` to run `git status` immediately!

### Summary

Using `source ~/.bashrc` is a quick way to apply changes you've made to your terminal settings without restarting anything. It helps keep your workflow smooth and efficient!
