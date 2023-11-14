#! /bin/bash

# rcnode
bash /home/robocomp/robocomp/tools/rcnode/rcnode.sh&

# brige
$HOME/robocomp/components/webots-bridge/bin/Webots2Robocomp $HOME/robocomp/components/webots-bridge/etc/config&

#joystick
$HOME/robocomp/components/robocomp-robolab/components/hardware/external_control/joystickpublish/bin/JoystickPublish $HOME/robocomp/components/robocomp-robolab/components/hardware/external_control/joystickpublish/etc/config_shadow&

