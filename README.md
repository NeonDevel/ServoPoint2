# ServoPoint2


 DIY project of simple **[DCC](https://en.wikipedia.org/wiki/Digital_Command_Control) Accessory decoder** for 2 servos and 2 relays.

This project is based on [ServoPoint2 by Paco Cańada](http://usuaris.tinet.cat/fmco/home_en.htm)

* Platform:  Microchip PIC12F675 and PIC12F629
* Software Tools: MPLAB X IDE (mpasmx)

### Major improvements:

* Long pressing SWICH for about 2.5s (depends on CV545) enters programming mode.  Both servos move to central positions, outputs are set on.
* Direct CV programming and address change is only possibile in programing mode.
* Capturing new address or short pressing SWITCH exits programming mode.

  

# Project Details

### Build Status

>finished

### Versioning

version 23.05.17

Given a version number DAY.MONTH.YEAR


## Project Usage

### Build

To build the ServoPoint2 project:

```
$ git clone https://github.com/NeonDevel/ServoPoint2.git
$ cd ServoPoint2/src
$ ./make_dcc2servo.BAT
```

## License
This program is distributed as is but WITHOUT ANY WARRANTY