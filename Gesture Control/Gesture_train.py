import cv2
import numpy as np
import os
import time
from cvzone.HandTrackingModule import HandDetector

# -----------------------------
# Initialization
# -----------------------------
cap = cv2.VideoCapture(0)
cap.set(3, 1280)
cap.set(4, 720)

detector = HandDetector(maxHands=1)
offset = 40
imgSize = 300

# -----------------------------
# Dataset Folders & Configuration Map
# Format: key_char: (class_name, folder_path)
# -----------------------------
CLASSES = {
    'a': ("Left", "Data/Left"),     # 'a' for Left
    'd': ("Right", "Data/Right"),   # 'd' for Right
    'w': ("Up", "Data/Up"),         # 'w' for Up
    's': ("Down", "Data/Down"),     # 's' for Down
    'f': ("Front", "Data/Front"),   # 'f' for Front
    'b': ("Back", "Data/Back")      # 'b' for Back
}

# Create directories and initialize file counters
class_data = {}
for key_char, (label, path) in CLASSES.items():
    os.makedirs(path, exist_ok=True)
    counter = len(os.listdir(path))
    class_data[label] = {
        'path': path,
        'counter': counter,
        'key': key_char
    }

# -----------------------------
# Continuous Capture Variables
# -----------------------------
captureMode = False
currentLabel = ""
lastSave = 0
saveInterval = 0.15  # Save every 150 ms (~6-7 images per second)

print("📸 Dataset Collector Online.")
print("Keys: [A] Left | [D] Right | [W] Up | [S] Down | [F] Front | [B] Back | [Q] Quit")

while True:
    success, img = cap.read()
    if not success:
        break

    hands, img = detector.findHands(img)
    imgWhite = None

    if hands:
        hand = hands[0]
        x, y, w, h = hand["bbox"]
        h_img, w_img, _ = img.shape

        x1 = max(0, x - offset)
        y1 = max(0, y - offset)
        x2 = min(w_img, x + w + offset)
        y2 = min(h_img, y + h + offset)

        imgCrop = img[y1:y2, x1:x2]

        if imgCrop.size != 0:
            imgWhite = np.ones((imgSize, imgSize, 3), np.uint8) * 255
            cropH, cropW = imgCrop.shape[:2]
            aspectRatio = cropH / cropW

            if aspectRatio > 1:
                scale = imgSize / cropH
                newW = int(cropW * scale)
                resized = cv2.resize(imgCrop, (newW, imgSize))
                xGap = (imgSize - newW) // 2
                imgWhite[:, xGap:xGap + newW] = resized
            else:
                scale = imgSize / cropW
                newH = int(cropH * scale)
                resized = cv2.resize(imgCrop, (imgSize, newH))
                yGap = (imgSize - newH) // 2
                imgWhite[yGap:yGap + newH, :] = resized

            # -----------------------------
            # Continuous Image Saving
            # -----------------------------
            if captureMode and currentLabel in class_data:
                currentTime = time.time()
                if currentTime - lastSave > saveInterval:
                    info = class_data[currentLabel]
                    file_path = f"{info['path']}/{info['counter']}.jpg"
                    
                    cv2.imwrite(file_path, imgWhite)
                    info['counter'] += 1
                    lastSave = currentTime

    # -----------------------------
    # Display Status Overlay
    # -----------------------------
    y_offset = 35
    for label, info in class_data.items():
        cv2.putText(
            img,
            f"{label} ({info['key'].upper()}): {info['counter']}",
            (10, y_offset),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 255),  # Red font
            2
        )
        y_offset += 30

    if captureMode:
        cv2.putText(
            img,
            f"RECORDING: {currentLabel}",
            (10, y_offset + 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),  # Yellow font
            2
        )

    cv2.imshow("Original", img)
    key = cv2.waitKey(1) & 0xFF

    # -----------------------------
    # Handle Dynamic Key Toggles
    # -----------------------------
    if key != 255:  # If a key was pressed
        pressed_char = chr(key).lower() if key < 256 else ""
        
        if pressed_char == 'q':
            break

        # Check key mapping
        for label, info in class_data.items():
            if pressed_char == info['key']:
                if captureMode and currentLabel == label:
                    captureMode = False
                    currentLabel = ""
                    print(f"⏹️ Stopped {label} Capture")
                else:
                    captureMode = True
                    currentLabel = label
                    print(f"🔴 Started {label} Capture")
                break

cap.release()
cv2.destroyAllWindows()