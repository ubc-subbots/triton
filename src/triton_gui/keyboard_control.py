import os
import signal
import record_all
from pynput import keyboard

class Keyboard_Controller():
    pid = 0
    def __init__(self):
        super().__init__()
        self._start()
        self.get_logger().info('Keyboard controller for recorded launched!')

    def _start(self):
        """
        Sets up the keyboard listeners
        """
        self.listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release
        )
        self.listener.start()
    
    # Handles lanuching the recorder for rosbag record_all.xml 
    def on_press(key):
        '''
            Handles pressing a key from the keyboard

            @param key: The character of the key pressed
        '''
        if 'char' in dir(key):
            # Import the recording python module here
            if key.char == 'r':
                # Update the PID so as to be able to kill it whenever we want
                pid = record_all.generate_launch_description()[1]
    
    def on_release(key):
        '''
            Handles key releases

            @param key: The character of the key pressed
        '''
        if 'char' in dir(key):
            # Use 0 as a flag to tell whether the PID has been updated at all
            if key.char == 'r' and pid != 0:
                os.kill(pid, signal.SIGTERM)

def main(args=None):
    # Instantiates the keyboard controller
    control = Keyboard_Controller()

if __name__ == '__main__':
    main()