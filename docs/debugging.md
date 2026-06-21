# Debugging

For GDB-based debugging, add the gdbserver prefix to the launch file:

```
gdbserver localhost:3000 --
```

Build with debug symbols and compile commands (for editor LSP support):

```bash
colcon build --packages-select radix_ros radix_msgs \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Then run the launch file with the prefix. The program will stop immediately after launch and wait for a debugger to attach.

Connect with GDB:

```bash
gdb
target remote localhost:3000
continue
```

`continue` runs the program until it hits a breakpoint or exception. Trigger the scenario you want to investigate (e.g. call a service), then:

```bash
bt
```

to get a backtrace showing where the crash or assertion occurred.
