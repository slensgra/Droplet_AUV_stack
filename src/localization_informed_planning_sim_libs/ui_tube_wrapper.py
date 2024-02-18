import serial
import time
import json

class UITubeController(object):
    pattern_name_to_byte_mapping = {
        'chase': 'c',
        'fade': 'f',
        'solid': 's'
    }

    def __init__(self):
        self.serial_port = '/dev/ttyUITube'
        self.ser = serial.Serial(
            port=self.serial_port,
            baudrate=9600,
            timeout=.1,
        )
        time.sleep(3.0)

        if not(self.ser.isOpen()):
            raise ValueError('Failed to open serial port {}'.format(self.serial_port))

    @classmethod
    def check_pattern(cls, pattern):
        if pattern not in cls.pattern_name_to_byte_mapping.keys():
            raise ValueError('Invalid pattern: {}. Valid choices are: {}'.format(pattern, UITubeController.pattern_name_to_byte_mapping.keys()))

    @classmethod
    def get_json_for_led_command(cls, pattern, r, g, b):
        return json.dumps({
            'type': 'led',
            'pattern': pattern,
            'r': r,
            'g': g,
            'b': b
        })

    @classmethod 
    def get_json_for_headlamp_command(cls, brightnesses):
        assert(len(brightnesses) == 4)

        return json.dumps({
            'brightnesses': brightnesses
        })


    def set_led(self, pattern, r, g, b):
        UITubeController.check_pattern(pattern)

        assert 0 <= r <= 255
        assert 0 <= g <= 255
        assert 0 <= b <= 255

        pattern_byte = UITubeController.pattern_name_to_byte_mapping[pattern]
        packet = bytearray([ord('>'), ord('l'), ord(pattern_byte), g, r, b, 0, ord('\n')])
        self.ser.write(packet)

    def set_headlamps(self, brightnesses):
        if len(brightnesses) != 4:
            raise ValueError('Expected 4 brightness values, got {}'.format(len(brightnesses)))

        for brightness in brightnesses:
            assert 0 <= brightness <= 255

        packet = bytearray([ord('>'), ord('h')] + brightnesses + [0, ord('\n')])
        self.ser.write(packet)

    def __del__(self):
        self.ser.close()


if __name__ == '__main__':
    ui_tube = UITubeController()

    print('Testing ui tube functionality')
    ui_tube.set_led('solid', 255, 0, 0)
    time.sleep(2.0)
    ui_tube.set_led('solid', 0, 255, 0)
    time.sleep(2.0)
    ui_tube.set_led('solid', 0, 0, 255)
    time.sleep(2.0)
    ui_tube.set_headlamps([0,0,0,0])
    time.sleep(2.0)
    ui_tube.set_headlamps([100,100,100,100])
    time.sleep(2.0)
    ui_tube.set_headlamps([0,0,0,0])
    print('Done!')