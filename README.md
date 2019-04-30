# Please Note
This file has been taken from https://github.com/Ubotica/telloCV/ and modified for optimization with our project goals.

# Tellocv tracker
Tracking code for the Tello drone. It uses opencv and tellopy to identify a blue ball in the scene and then send commands to the drone.
It is written in python3 and implemented using Visual Studio Code 

## Installation
You need to have opencv installed and the following python modules for the tello and cv2:
apt

```
sudo apt install sudo libopencv-dev python3-opencv
```

pip3:

```
sudo pip3 install imutils pynput
```

you need to build tellopy from source as the :
Or install from the source code.
```
git clone https://github.com/hanyazou/TelloPy
cd TelloPy
python setup.py bdist_wheel
pip install dist/tellopy-*.dev*.whl --upgrade
```
requirements.txt is included for all the library requirements needed for this code.

# Flight rules
- Although tellos are very safe to operate, wear safety glasses as an added precaution
- Do not fly over people
- Memorize the controls *before* taking off
- Always be ready to land the drone (backspace)
- If the program crashes restart it to regain control
- if drone is going out of control just hit it and it will turn off.

## Tello lights

- flashing blue - charging
- solid blue - charged
- flashing purple - booting up
- flashing yellow fast - wifi network set up, waiting for connection
- flashing yellow - User connected
 
 ## Controls 
 'w': 'forward',
 's': 'backward',
 'a': 'left',
 'd': 'right',
 'Key.space': 'up',
 'Key.shift': 'down',
 'Key.shift_r': 'down',
 'q': 'counter_clockwise',
 'e': 'clockwise',
 'i': lambda speed: self.drone.flip_forward(),
 'k': lambda speed: self.drone.flip_back(),
 'j': lambda speed: self.drone.flip_left(),
 'l': lambda speed: self.drone.flip_right(),
 
 # arrow keys for fast turns and altitude adjustments
 'Key.left': lambda speed: self.drone.counter_clockwise(speed),
 'Key.right': lambda speed: self.drone.clockwise(speed),
 'Key.up': lambda speed: self.drone.up(speed),
 'Key.down': lambda speed: self.drone.down(speed),
 'Key.tab': lambda speed: self.drone.takeoff(),
 'Key.backspace': lambda speed: self.drone.land(),
 'p': lambda speed: self.palm_land(speed),
 't': lambda speed: self.toggle_tracking(speed),
 'r': lambda speed: self.toggle_recording(speed),
 'z': lambda speed: self.toggle_zoom(speed),
 'Key.enter': lambda speed: self.take_picture(speed)
