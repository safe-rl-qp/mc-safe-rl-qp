# mc_unitree2
Interface between [Unitree robots](https://github.com/unitreerobotics/unitree_ros2/tree/master/robots) and [mc_rtc](https://jrl-umi3218.github.io/mc_rtc). Provides connectivity with [Go2](https://www.unitree.com/products/go2/) robots.

## 1. Required dependencies

 - [mc_rtc](https://jrl-umi3218.github.io/mc_rtc/)
 - [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)

## 2. Install dependencies

### unitree_sdk2
 - Install the include files
```
$ git clone https://github.com/unitreerobotics/unitree_sdk2
$ mkdir build
$ cmake ..
$ make ; make install
```
## 3. Install this project

### Build instructions

```
$ cd src
$ git clone https://github.com/mc_unitree2
$ mkdir build
$ cd build
$ cmake ..
$ make ; make install
```

## 4. Usage
Run the program:

```
$ MCControlUnitree --conf <install path/etc/mc_unitree/mc_rtc.yaml> --network <name of network adaptor>
```

```
# Interface specific parameters (mc_unitree)
Unitree:
  network-interface: # Name of network adaptor
```
