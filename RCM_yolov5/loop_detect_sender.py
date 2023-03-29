import threading
import cv2
from detector import Detector
import time
from tcp_sender import Sender

class Cap:
    def __init__(self, cam_id=0):
        self.camera = cv2.VideoCapture(cam_id)
        ret, self.img = self.camera.read()
        print("INIT")
        if not ret:
            print("READ IMG ERROR")
        self.mtx = threading.Lock()
    
    def stop(self):
        self.stop_flag = True
        self.thread_run.join()
    
    def start(self):
        self.stop_flag = False
        self.thread_run = threading.Thread(target=self.__run)
        self.thread_run.start()
    
    def __run(self):
        while not self.stop_flag:
            self.mtx.acquire()
            ret, self.img = self.camera.read()
            self.mtx.release()
            if ret:
                time.sleep(0.003)
            else:
                print("CAM_ERROR")

    def read(self):
        self.mtx.acquire()
        img0 = self.img.copy()
        self.mtx.release()
        
        return img0
    
if __name__ == '__main__':
    cam = Cap(cam_id=0)
    cam.start()
    detector = Detector(classes=0, conf_thres=0.75, view_img=True)
    sd = Sender('127.0.0.1')
    windows_name = "monitor"
    cv2.namedWindow(windows_name, cv2.WINDOW_NORMAL)
    cv2.moveWindow(windows_name, 1920, 0)
    cv2.setWindowProperty(windows_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    try:
        while True:       
            img = cam.read()
            
            det, im0 = detector.run(img)
            
            cv2.imshow(windows_name, im0)
            if cv2.waitKey(1) == 27:
                cam.stop()
                break
            
            px = 0
            py = 0
            max_conf = 0
            for *xyxy, conf, cls in reversed(det):
                if conf < max_conf:
                    continue
                max_conf = conf

                cx = (xyxy[0] + xyxy[2]) / 2
                cy = (xyxy[1] + xyxy[3]) / 2
                
                px = -(cx.item() / 320 - 1)
                py = cy.item() / 240 - 1
                
                # print(f"classes: {cls}, conf: {conf}, center: ({cx}, {cy}) -> ({px}, {py})")
            
            sd.sendvec(px, py)
    except KeyboardInterrupt:
        print("Keyboard Interrupt!")
    except ConnectionResetError:
        print("Connection Lost!")
    cam.stop()
    sd.close()
