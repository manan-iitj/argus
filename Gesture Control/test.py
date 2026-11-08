import os

os.environ["MEDIAPIPE_DISABLE_GPU"] = "1"

import cv2
import numpy as np
import socket
import speech_recognition as sr

from cvzone.HandTrackingModule import HandDetector
from cvzone.ClassificationModule import Classifier

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(1.0)

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    raise RuntimeError(
        "Could not open webcam. Check macOS camera permissions."
    )

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

detector = HandDetector(
    maxHands=1,
    detectionCon=0.5,
    minTrackCon=0.5
)

classifier = Classifier(
    "keras_model.h5",
    "labels.txt"
)

labels = [
    "Back",
    "Down",
    "Front",
    "Left",
    "Right",
    "Up"
]

offset = 40
imgSize = 300

currentGesture = ""
stableFrames = 0

requiredFrames = 20
confidenceThreshold = 0.97

last_sent_command = None
last_voice_command = "None"


def send_udp_command(command_str, source="Gesture"):

    global last_sent_command

    command_str = command_str.upper()

    try:

        sock.sendto(
            command_str.encode("utf-8"),
            (UDP_IP, UDP_PORT)
        )

        if command_str != last_sent_command:

            print(
                f"📡 [{source}] "
                f"Sent Command to GCP "
                f"({UDP_IP}:{UDP_PORT}): "
                f"{command_str}"
            )

            last_sent_command = command_str

    except OSError as e:

        print(
            f"⚠️ UDP transmission error: {e}"
        )


voice_keywords = {

    "launch": "LAUNCH",
    "take off": "LAUNCH",
    "takeoff": "LAUNCH",

    "land": "LAND",

    "inspect": "INSPECT",
    "hold": "INSPECT"
}


def voice_callback(recognizer, audio):

    global last_voice_command

    try:

        text = recognizer.recognize_google(audio).lower()

        print(
            f"🗣️ [Online Voice API] "
            f"Heard: '{text}'"
        )

        for key, command in voice_keywords.items():

            if key in text:

                last_voice_command = command

                send_udp_command(
                    command,
                    source="VOICE"
                )

                break

    except sr.UnknownValueError:

        pass

    except sr.RequestError as e:

        print(
            f"⚠️ Google Online Speech API error: {e}"
        )


recognizer = sr.Recognizer()

mic = sr.Microphone()

with mic as source:

    print(
        "🎙️ Calibrating microphone "
        "for ambient noise..."
    )

    recognizer.adjust_for_ambient_noise(
        source,
        duration=1.0
    )


stop_listening = recognizer.listen_in_background(
    mic,
    voice_callback,
    phrase_time_limit=3
)

print(
    "🌐 Online Voice Control Active!"
)

print(
    "🎮 Gesture Control Active!"
)

print(
    f"📡 GCP Target: {UDP_IP}:{UDP_PORT}"
)


try:

    while True:

        success, img = cap.read()

        if not success:

            print(
                "⚠️ Failed to read webcam frame."
            )

            break

        hands, img = detector.findHands(
            img,
            draw=True
        )

        predictionLabel = "No Hand"
        confidence = 0.0

        if hands:

            hand = hands[0]

            x, y, w, h = hand["bbox"]

            H, W, _ = img.shape

            x1 = max(
                0,
                x - offset
            )

            y1 = max(
                0,
                y - offset
            )

            x2 = min(
                W,
                x + w + offset
            )

            y2 = min(
                H,
                y + h + offset
            )

            imgCrop = img[
                y1:y2,
                x1:x2
            ]

            if imgCrop.size != 0:

                imgWhite = np.ones(
                    (
                        imgSize,
                        imgSize,
                        3
                    ),
                    np.uint8
                ) * 255

                cropH, cropW = (
                    imgCrop.shape[:2]
                )

                aspectRatio = (
                    cropH / cropW
                )

                if aspectRatio > 1:

                    scale = (
                        imgSize / cropH
                    )

                    newW = max(
                        1,
                        int(scale * cropW)
                    )

                    imgResize = cv2.resize(
                        imgCrop,
                        (
                            newW,
                            imgSize
                        )
                    )

                    xGap = (
                        imgSize - newW
                    ) // 2

                    imgWhite[
                        :,
                        xGap:xGap + newW
                    ] = imgResize

                else:

                    scale = (
                        imgSize / cropW
                    )

                    newH = max(
                        1,
                        int(scale * cropH)
                    )

                    imgResize = cv2.resize(
                        imgCrop,
                        (
                            imgSize,
                            newH
                        )
                    )

                    yGap = (
                        imgSize - newH
                    ) // 2

                    imgWhite[
                        yGap:yGap + newH,
                        :
                    ] = imgResize

                prediction, index = (
                    classifier.getPrediction(
                        imgWhite,
                        draw=False
                    )
                )

                if 0 <= index < len(labels):

                    predictionLabel = labels[index]

                    confidence = float(
                        prediction[index]
                    )

                if (
                    confidence
                    >= confidenceThreshold
                ):

                    if (
                        predictionLabel
                        == currentGesture
                    ):

                        stableFrames += 1

                    else:

                        currentGesture = (
                            predictionLabel
                        )

                        stableFrames = 1

                    if (
                        stableFrames
                        >= requiredFrames
                    ):

                        command = (
                            predictionLabel.upper()
                        )

                        send_udp_command(
                            command,
                            source="GESTURE"
                        )

                        stableFrames = (
                            requiredFrames
                        )

                else:

                    stableFrames = 0
                    currentGesture = ""

        else:

            stableFrames = 0
            currentGesture = ""

        cv2.putText(
            img,
            f"Gesture: {predictionLabel}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 0, 0),
            2
        )

        cv2.putText(
            img,
            f"Confidence: {confidence * 100:.1f}%",
            (20, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 255, 0),
            2
        )

        cv2.putText(
            img,
            f"Stable: {stableFrames}/{requiredFrames}",
            (20, 120),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 0, 255),
            2
        )

        cv2.putText(
            img,
            f"Voice: {last_voice_command}",
            (20, 160),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 0),
            2
        )

        cv2.imshow(
            "ARGUS - Hybrid Gesture + Voice Control",
            img
        )

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):

            break

finally:

    try:

        stop_listening(
            wait_for_stop=False
        )

    except Exception:
        pass

    cap.release()

    cv2.destroyAllWindows()

    sock.close()

    print(
        "\n🛑 ARGUS control system stopped."
    )
