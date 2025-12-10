# Debugging


For real debugging i didn't fine a better way than the following:

Add the prefix to the launch file (see launch file):
```
gdbserver localhost:3000 --
```

And add RelWithDebInfo to the colcon/cmake build (EXPORT COMPILE COMMANDS is for editor lsp support):

```
colcon build --packages-select radix_ros radix_msgs --cmake-args-DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Then run the launch file with the prefix.
This enables remote debugging with gdb.
You will notice that the program will stop right away.

To debug after launching:

```
gdb
```

then to connect to the remote gdbserver (the program):

```
target remote localhost:3000
```

then to run the program:


```
continue
```

this will run the program until it hits a breakpoint/exception.


You can now call a service for example, or otherwise produce the crash that lead you to debugging.
Then

```
bt
```

to get the backtrace (where the actual crash happened).


Put it in chatgpt or read actually read it :).
