# 3070grp5

This is the ROBOCON 3070 Group 5 project repository.

The `grp5_node` will create 4 seperate `Float32` channels of name

```
whlspd1
whlspd2
whlspd3
whlspd4
```

of which is subscribed by `Rm_motor_speed_control`.

To initialize DDS-Agent:

```
MicroXRCEAgent serial --dev /dev/ttyACM1 -b 115200 -v0
```
