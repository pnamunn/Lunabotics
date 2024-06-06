# How the Logitech F310 gamepad acts

On D, with mode light on:
    left joystick has x and y axis values of [-1,1] with 0 being neutral
    right joystick has x and y axis values of [0,255] with 127 or 128 being neutral

## USE THIS ONE :
On D, with mode light off:
    both joysticks have x and y axis values of [0,255] with 127 or 128 being neutral
        in joy, this becomes [-1,1] 32 bit float with 16 decimal places with 0 being neutral
    all buttons and triggers are boolean

On X, with mode light on:
    left joystick has x and y axis values of [-1,1] with 0 being neutral
    right joystick has x and y axis values of [-32768, 32767] with 128 being neutral for x, -129 being neutral for y

On X, with mode light off:
    both joysticks have x and y axis values of [-32768, 32767] with 128 being neutral for x, -129 being neutral for y


## On D, with mode light off controller mappings:

msg.buttons = {
    [0]     X
    [1]     A
    [2]     B
    [3]     Y
    [4]     LB
    [5]     RB
    [6]     LT
    [7]     RT
    [8]     back
    [9]     start
    [10]    Left stick press
    [11]    Right stick press
} 

msg.axes = {
    [0]     Left stick x axis
    [1]     Left stick y axis
    [2]     Right stick x axis
    [3]     Right stick y axis
    [4]     D pad left & right
    [5]     D pad up & down
}
