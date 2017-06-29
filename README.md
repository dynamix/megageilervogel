# megageilervogel


                                   .-.  .--''` )
                                _ |  |/`   .-'`
                               ( `\      /`
                               _)   _.  -'._
                             /`  .'     .-.-;
                             `).'      /  \  \
                            (`,        \_o/_o/__
                             /           .-''`  ``'-.
                             {         /` ,___.--''`
                             {   ;     '-. \ \
           _   _             {   |'-....-`'.\_\
          / './ '.           \   \          `"`
       _  \   \  |            \   \
      ( '-.J     \_..----.._ __)   `\--..__
     .-`                    `        `\    ''--...--.
    (_,.--""`/`         .-             `\       .__ _)
            |          (                 }    .__ _)
            \_,         '.               }_  - _.'
               \_,         '.            } `'--'
                  '._.     ,_)          /
                     |    /           .'
                      \   |    _   .-'
                       \__/;--.||-'
                        _||   _||__   __
                 _ __.-` "`)(` `"  ```._)
                (_`,-   ,-'  `''-.   '-._)
               (  (    /          '.__.'
                `"`'--'


## Setup

- needs a Teensy 3.6
- needs a RaspperyPi Zero W
- Arduino 1.8.3
- the latest Teensy for Arduino

Add the following to your boards.txt (on OSX in /Applications/Arduino.app/Contents/Java/hardware/teensy/avr/boards.txt)

```
teensy36.build.fcpu=180000000
teensy36.build.usbtype=USB_SERIAL
teensy36.build.keylayout=US_ENGLISH
teensy36.build.flags.optimize=-Os
teensy36.build.flags.ldspecs=--specs=nano.specs
```

# Build

make

# Deploy

make deploy
