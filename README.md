             _/|       |\_
            /  |       |  \
           |    \     /    |
           |  \ /     \ /  |
           | \  |     |  / |
           | \ _\_/^\_/_ / |
           |    --\//--    |
            \_  \     /  _/
              \__  |  __/
                 \ _ /
                _/   \_   Project Phoenix ;-)
               / _/|\_ \  
                /  |  \   
                 / v \


# megageilervogel

## Prerequisites

- Arduino 1.8.3
- latest Teensyduino
- a Teensy 3.6
- a RaspperyPi Zero W

## Setup

Add the following to your boards.txt (on OSX in /Applications/Arduino.app/Contents/Java/hardware/teensy/avr/boards.txt)

```
teensy36.build.fcpu=180000000
teensy36.build.usbtype=USB_SERIAL
teensy36.build.keylayout=US_ENGLISH
teensy36.build.flags.optimize=-Os
teensy36.build.flags.ldspecs=--specs=nano.specs
```

# Build

```
make
```

# Deploy

Over the air deployment relies on:
- the PI is connected to the local network
- the PI and the Teensy are correctly connected
- the teensyloader is available on the PI

```
make deploy
```
