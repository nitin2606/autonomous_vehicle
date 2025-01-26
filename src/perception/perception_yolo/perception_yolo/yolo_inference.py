from ultralytics import YOLO
import time 

class YoloInference:
    
    def __init__(self, model_path):

        self.model = YOLO(model=model_path)
        self.results = None
        self.processed_image = None
    
    def get_processed_frame(self, frame):

        self.results = self.model(frame)
        self.processed_image = self.results[0].plot()

        return self.results, self.processed_image

    
    

        