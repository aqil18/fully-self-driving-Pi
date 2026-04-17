import cv2
import numpy as np
import random
import glob
import os

DATASET_DIR = os.path.join(os.path.dirname(__file__), "datasets")

def get_lateral_offset(frame):
    # crop to bottom 70% - only care about near track
    roi = frame[int(frame.shape[0] * 0.3):, :]

    # convert to grayscale or HSV depending on tape color
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    # threshold to isolate tape
    _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    # find contours of tape
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None  # lost the line

    # get centroid of largest contour
    c = max(contours, key=cv2.contourArea)
    M = cv2.moments(c)
    cx = int(M['m10'] / M['m00'])

    # offset from center of frame
    frame_center = frame.shape[1] // 2
    return cx - frame_center


def visualize(frame):
    """
    Run line detection on a frame and return an annotated BGR image showing:
    - The ROI crop boundary
    - All detected contours (green)
    - The largest contour filled (blue)
    - The centroid (red dot + crosshair)
    - The frame centre line (yellow)
    - The lateral offset printed on screen
    """
    h, w = frame.shape[:2]
    roi_top = int(h * 0.3)
    roi = frame[roi_top:, :]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    out = frame.copy()

    # ROI divider
    cv2.line(out, (0, roi_top), (w, roi_top), (0, 255, 255), 1)

    # Centre line
    frame_center = w // 2
    cv2.line(out, (frame_center, roi_top), (frame_center, h), (0, 255, 255), 1)

    if not contours:
        cv2.putText(out, "NO LINE DETECTED", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return out

    # All contours in green (offset to ROI)
    shifted = [c + np.array([0, roi_top]) for c in contours]
    cv2.drawContours(out, shifted, -1, (0, 200, 0), 1)

    # Largest contour filled blue
    largest = max(contours, key=cv2.contourArea)
    largest_shifted = largest + np.array([0, roi_top])
    cv2.drawContours(out, [largest_shifted], -1, (200, 100, 0), 2)

    M = cv2.moments(largest)
    if M['m00'] == 0:
        cv2.putText(out, "ZERO MOMENT", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return out

    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00']) + roi_top

    # Centroid
    cv2.circle(out, (cx, cy), 6, (0, 0, 255), -1)
    cv2.line(out, (cx, cy - 12), (cx, cy + 12), (0, 0, 255), 1)
    cv2.line(out, (cx - 12, cy), (cx + 12, cy), (0, 0, 255), 1)

    # Offset line
    cv2.line(out, (frame_center, cy), (cx, cy), (255, 0, 255), 2)

    offset = cx - frame_center
    label = f"offset: {offset:+d} px"
    cv2.putText(out, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    return out


if __name__ == "__main__":
    image_paths = glob.glob(os.path.join(DATASET_DIR, "**", "images", "*.jpg"), recursive=True)
    if not image_paths:
        raise FileNotFoundError(f"No images found under {DATASET_DIR}")

    print(f"Found {len(image_paths)} images. Press any key for next, 'q' to quit.")

    random.shuffle(image_paths)
    for path in image_paths:
        frame = cv2.imread(path)
        if frame is None:
            continue

        annotated = visualize(frame)
        offset = get_lateral_offset(frame)
        title = f"{os.path.basename(path)}  |  offset={offset}"
        display = cv2.resize(annotated, (0, 0), fx=3, fy=3, interpolation=cv2.INTER_NEAREST)
        cv2.imshow(title, display)

        key = cv2.waitKey(0) & 0xFF
        cv2.destroyAllWindows()
        if key == ord('q'):
            break