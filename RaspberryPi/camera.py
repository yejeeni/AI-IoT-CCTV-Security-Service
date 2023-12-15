import time
import io
import threading
import picamera


class Camera(object):
    thread = None
    frame = None
    last_access = 0

    def initialize(self):
        if Camera.thread is None:
            Camera.thread = threading.Thread(target=self._thread)
            Camera.thread.start()

            while self.frame is None:
                time.sleep(0)

    def get_frame(self):
        Camera.last_access = time.time()
        self.initialize()
        return self.frame
        
    def capture_pi(self):
        with picamera.PiCamera() as camera:
            camera.capture('test.jpg')

    @classmethod
    def _thread(cls):
        count = 0
        with picamera.PiCamera() as camera:
            camera.resolution = (320, 240)
            camera.hflip = True
            camera.vflip = True

            camera.start_preview()
            time.sleep(2)

            stream = io.BytesIO()
            for foo in camera.capture_continuous(stream, 'jpeg',
                                                 use_video_port=True):
                stream.seek(0)
                cls.frame = stream.read()
                
                if count % 100 == 0: ##
                    camera.capture(f'test.jpg')
                    count = 1
                print(count)

                count += 1 ##

                stream.seek(0)
                stream.truncate()

                if time.time() - cls.last_access > 10:
                    break
        cls.thread = None
