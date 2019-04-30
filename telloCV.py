"""
tellotracker:
Allows manual operation of the drone and demo tracking mode.

Requires mplayer to record/save video.

Controls:
- tab to lift off
- WASD to move the drone
- space/shift to ascend/descent slowly
- Q/E to yaw slowly
- arrow keys to ascend, descend, or yaw quickl
- backspace to land, or P to palm-land
- enter to take a picture
- R to start recording video, R again to stop recording
  (video and photos will be saved to a timestamped file in ~/Pictures/)
- Z to toggle camera zoom state
  (zoomed-in widescreen or high FOV 4:3)
- T to toggle tracking  
@author Leonie Buckley, Saksham Sinha and Jonathan Byrne
@copyright 2018 see license file for details
"""
import time
import datetime
import os
import tellopy
import numpy
import av   
import cv2.cv2 as cv2
from pynput import keyboard
from tracker import Tracker
import threading #Python library that allows you to create multiple threads to run multiple functions at the same time. 

current_frame = None # initialize global variable current_frame as None until it is changed in frame_save #numpy.array([[[0,0,0]]])  #initialize global variable as a single black pixel
exiting = False

def frame_save(tellotrack):
    """ Stores frames from the Tello VideoStream to a temporary variable """
    global current_frame # allows the global variable defined above to be changed inside this function
    global exiting
    for packet in tellotrack.container.demux((tellotrack.vid_stream,)):
        try:
            for frame in packet.decode():
                current_frame = frame
        except Exception as e:
            print("ERROR!!! {}".format(e))
    exiting = True
    print("EXITING FRAME_SAVE THREAD!")
            
def main():
    """ Create a tello controller and show the video feed."""
    global current_frame # allows the global variable defined above to be changed inside this function
    global exiting
    tellotrack = TelloCV()
    threading.Thread(target=frame_save, args=[tellotrack]).start() #Create a thread that runs the function frame_save while main runs in the current thread
    
    while not exiting:
        if not current_frame is None: #don't run until there is a new current_frame
            image = tellotrack.process_frame(current_frame)
            cv2.imshow('tello', image)
            _ = cv2.waitKey(1) & 0xFF
    print("EXITING MAIN THREAD!")

class TelloCV(object):
    """
    TelloTracker builds keyboard controls on top of TelloPy as well
    as generating images from the video stream and enabling opencv support
    """

    def __init__(self):
        self.prev_flight_data = None
        self.record = False
        self.tracking = False
        self.keydown = False
        self.date_fmt = '%Y-%m-%d_%H%M%S'
        self.speed = 50
        self.drone = tellopy.Tello()
        self.init_drone()
        self.init_controls()

        # container for processing the packets into frames
        self.container = av.open(self.drone.get_video_stream())
        self.vid_stream = self.container.streams.video[0]
        self.out_file = None
        self.out_stream = None
        self.out_name = None
        self.start_time = time.time()

        # tracking a color  
        #Assign a range of RGB values for the drone to look for
        #green_lower = (30, 50, 50) 
        #green_upper = (80, 255, 255)
        #red_lower = (0, 50, 50)
        # red_upper = (20, 255, 255)
        # blue_lower = (0, 0, 130)
        # upper_blue = (150, 220, 255)
        color_lower = (90, 50, 50)
        color_upper =(110, 255, 255,)
        self.track_cmd = ""
        self.track_speed = 0
        self.init_PID()
        self.colortracker = Tracker(self.vid_stream.height,
                               self.vid_stream.width,
                               color_lower, color_upper)

    def init_drone(self):
        """Connect, uneable streaming and subscribe to events"""
        # self.drone.log.set_level(2)
        self.drone.connect()
        self.drone.start_video()
        self.drone.subscribe(self.drone.EVENT_FLIGHT_DATA,
                             self.flight_data_handler)
        self.drone.subscribe(self.drone.EVENT_FILE_RECEIVED,
                             self.handle_flight_received)


    def on_press(self, keyname):
        """handler for keyboard listener"""
        if self.keydown:
            return
        try:
            self.keydown = True
            keyname = str(keyname).strip('\'')
            print('+' + keyname)
            if keyname == 'Key.esc':
                self.drone.quit()
                exit(0)
            if keyname in self.controls:
                key_handler = self.controls[keyname]
                if isinstance(key_handler, str):
                    getattr(self.drone, key_handler)(self.speed)
                else:
                    key_handler(self.speed)
        except AttributeError:
            print('special key {0} pressed'.format(keyname))

    def on_release(self, keyname):
        """Reset on key up from keyboard listener"""
        self.keydown = False
        keyname = str(keyname).strip('\'')
        print('-' + keyname)
        if keyname in self.controls:
            key_handler = self.controls[keyname]
            if isinstance(key_handler, str):
                getattr(self.drone, key_handler)(0)
            else:
                key_handler(0)

    def init_controls(self):
        """Define keys and add listener"""
        self.controls = {
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
        }
        self.key_listener = keyboard.Listener(on_press=self.on_press,
                                              on_release=self.on_release)
        self.key_listener.start()
        # self.key_listener.join()

    #We want to use a feedback control loop to direct the drone towards the recognized object from tracker.py
    #to do this we will use a PID controller.   
        #find an optimal Vx and Vy using a PID
        # distance = 100 # setpoint, pixels
        # Vx = 0      #manipulated variable
        # Vy = 0      #manipulated variable 
    def init_PID(self):
        def proportional():
            Vx = 0
            Vy = 0
            prev_time = time.time()
            Ix = 0
            Iy = 0
            ex_prev = 0
            ey_prev = 0
            while True:
                #yield an x and y velocity from xoff, yoff, and distance
                xoff, yoff, distance = yield Vx, Vy

                #PID Calculations
                ex = xoff - distance
                ey = yoff - distance
                current_time = time.time()
                delta_t = current_time - prev_time
                
                #Control Equations, constants are adjusted as needed
                Px = 0.1*ex
                Py = 0.1*ey
                Ix = Ix + -0.0*ex*delta_t
                Iy = Iy + -0.0*ey*delta_t
                Dx = 0.01*(ex - ex_prev)/(delta_t)
                Dy = 0.01*(ey - ey_prev)/(delta_t)

                Vx = Px + Ix + Dx
                Vy = Py + Iy + Dy

                #update the stored data for the next iteration
                ex_prev = ex
                ey_prev = ey
                prev_time = current_time
        self.PID = proportional()
        self.PID.send(None)    


    def process_frame(self, frame):
        """convert frame to cv2 image and show"""
        image = cv2.cvtColor(numpy.array(
            frame.to_image()), cv2.COLOR_RGB2BGR)
        image = self.write_hud(image)
        if self.record:
            self.record_vid(frame)
        distance = 0
        xoff, yoff = self.colortracker.track(image)
        image = self.colortracker.draw_arrows(image)
        Vx,Vy=self.PID.send([xoff, yoff, distance])        
        print("TARGET_V: ({Vx},{Vy})".format(Vx=Vx,Vy=Vy)) # Print statement to ensure Vx and Vy are reasonable values (<50)   
                
        # Create a loop to implement the Vx and Vy as a command to move the drone accordingly
        cmd = "" 
        speed = 0

        if self.tracking:
            if abs(Vx) > abs(Vy):
                if Vx > 0:
                    cmd = "right"
                    speed = abs(Vx)
                elif Vx < 0:
                    cmd = "left"
                    speed = abs(Vx)
            else:
                if Vy > 0:
                    cmd = "up"
                    speed = abs(Vy)
                elif Vy < 0:
                    cmd = "down"
                    speed = abs(Vy)
            
        #Original Code:
        # if self.tracking:
        #     if xoff < -distance:
        #         cmd = "counter_clockwise"
        #     elif xoff > distance:
        #         cmd = "clockwise"
        #     elif yoff < -distance:
        #         cmd = "down"
        #     elif yoff > distance:
        #         cmd = "up"
        #     else:
        #         if self.track_cmd is not "":
        #             getattr(self.drone, self.track_cmd)(0)
        #             self.track_cmd = ""

        print(self.track_cmd)
        if cmd is not self.track_cmd or speed != self.track_speed: #!= means not equal to
            if cmd is not "":
                print("track command:", cmd)
                print("track speed:", speed)
                getattr(self.drone, cmd)(speed)
                self.track_cmd = cmd
                self.track_speed = speed
            else:
                if self.track_cmd is not "":
                    getattr(self.drone, self.track_cmd)(0)
                    self.track_cmd = ""
                    print("STOPPING!!!!!!!!")
                    getattr(self.drone, "down")(0) #Stop the drone
                    getattr(self.drone, "right")(0) #Stop the drone
        return image

    def write_hud(self, frame):
        """Draw drone info, tracking and record on frame"""
        stats = self.prev_flight_data.split('|')
        stats.append("Tracking:" + str(self.tracking))
        if self.drone.zoom:
            stats.append("VID")
        else:
            stats.append("PIC")
        if self.record:
            diff = int(time.time() - self.start_time)
            mins, secs = divmod(diff, 60)
            stats.append("REC {:02d}:{:02d}".format(mins, secs))

        for idx, stat in enumerate(stats):
            text = stat.lstrip()
            cv2.putText(frame, text, (0, 30 + (idx * 30)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (255, 0, 0), lineType=30)
        return frame

    def toggle_recording(self, speed):
        """Handle recording keypress, creates output stream and file"""
        if speed == 0:
            return
        self.record = not self.record

        if self.record:
            datename = [os.getenv('HOME'), datetime.datetime.now().strftime(self.date_fmt)]
            self.out_name = '{}/Pictures/tello-{}.mp4'.format(*datename)
            print("Outputting video to:", self.out_name)
            self.out_file = av.open(self.out_name, 'w')
            self.start_time = time.time()
            self.out_stream = self.out_file.add_stream(
                'mpeg4', self.vid_stream.rate)
            self.out_stream.pix_fmt = 'yuv420p'
            self.out_stream.width = self.vid_stream.width
            self.out_stream.height = self.vid_stream.height

        if not self.record:
            print("Video saved to ", self.out_name)
            self.out_file.close()
            self.out_stream = None

    def record_vid(self, frame):
        """
        convert frames to packets and write to file
        """
        new_frame = av.VideoFrame(
            width=frame.width, height=frame.height, format=frame.format.name)
        for i in range(len(frame.planes)):
            new_frame.planes[i].update(frame.planes[i])
        pkt = None
        try:
            pkt = self.out_stream.encode(new_frame)
        except IOError as err:
            print("encoding failed: {0}".format(err))
        if pkt is not None:
            try:
                self.out_file.mux(pkt)
            except IOError:
                print('mux failed: ' + str(pkt))

    def take_picture(self, speed):
        """Tell drone to take picture, image sent to file handler"""
        if speed == 0:
            return
        self.drone.take_picture()

    def palm_land(self, speed):
        """Tell drone to land"""
        if speed == 0:
            return
        self.drone.palm_land()

    def toggle_tracking(self, speed):
        """ Handle tracking keypress"""
        if speed == 0:  # handle key up event
            return
        self.tracking = not self.tracking
        print("tracking:", self.tracking)
        return

    def toggle_zoom(self, speed):
        """
        In "video" mode the self.drone sends 1280x720 frames.
        In "photo" mode it sends 2592x1936 (952x720) frames.
        The video will always be centered in the window.
        In photo mode, if we keep the window at 1280x720 that gives us ~160px on
        each side for status information, which is ample.
        Video mode is harder because then we need to abandon the 16:9 display size
        if we want to put the HUD next to the video.
        """
        if speed == 0:
            return
        self.drone.set_video_mode(not self.drone.zoom)

    def flight_data_handler(self, event, sender, data):
        """Listener to flight data from the drone."""
        text = str(data)
        if self.prev_flight_data != text:
            self.prev_flight_data = text

    def handle_flight_received(self, event, sender, data):
        """Create a file in ~/Pictures/ to receive image from the drone"""
        path = '%s/Pictures/tello-%s.jpeg' % (
            os.getenv('HOME'),
            datetime.datetime.now().strftime(self.date_fmt))
        with open(path, 'wb') as out_file:
            out_file.write(data)
        print('Saved photo to %s' % path)


if __name__ == '__main__':
    main()
