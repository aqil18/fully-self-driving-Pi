import cv2
import np    

class PreProcessor():
        def __init__(self):
                ### Image preprocessing
                self.out_h: int = 66
                self.out_w: int = 200

                
        def preprocess(self, bgr: np.ndarray) -> np.ndarray:
                """
                Simple & safe preprocess:
                - crop lower 60% (road-ish)
                - resize to (out_w, out_h)
                - convert BGR->RGB
                - normalize to [0,1]
                """
                h, w = bgr.shape[:2]

                # crop: keep bottom 60%
                top = int(h * 0.40)
                cropped = bgr[top:, :]
        
                # Resized
                resized = cv2.resize(cropped, (self.out_w, self.out_h), interpolation=cv2.INTER_AREA)

                # RGB AND normalizes by dividing by 255
                rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.float64) / 255.0
                
                return rgb