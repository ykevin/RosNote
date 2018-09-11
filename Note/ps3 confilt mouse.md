#修改/usr/share/X11/xorg.conf.d/10-evdev.conf 增加

Section "InputClass"
    Identifier "joystick catchall"
    MatchIsJoystick "on"
    MatchDevicePath "/dev/input/event*"
    Driver "joystick"
    Option "StartKeysEnabled" "False"
    Option "StartMouseEnabled" "False"
EndSection
