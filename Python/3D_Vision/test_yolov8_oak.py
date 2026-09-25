import argparse
from pathlib import Path

import cv2
import depthai as dai

#python test_yolov8_oak.py --model 416
# python test_yolov8_oak.py --model 640

LABELS = [
    "Back",
    "Front",
    "Gripper",
    "left_side",
    "right_side",
    "upright",
    "upside_down",
]


def create_pipeline(model_size: int, blob_path: Path):
    pipeline = dai.Pipeline()

    cam_rgb = pipeline.create(dai.node.ColorCamera)
    mono_left = pipeline.create(dai.node.MonoCamera)
    mono_right = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    detection_nn = pipeline.create(dai.node.YoloSpatialDetectionNetwork)

    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_nn = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")
    xout_nn.setStreamName("detections")


    cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
    cam_rgb.setPreviewSize(model_size, model_size)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
    mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
    stereo.setOutputSize(
        mono_left.getResolutionWidth(),
        mono_left.getResolutionHeight(),
    )
    stereo.setSubpixel(True)

    detection_nn.setBlobPath(str(blob_path))
    detection_nn.setConfidenceThreshold(0.5)
    detection_nn.setNumClasses(len(LABELS))
    detection_nn.setCoordinateSize(4)
    detection_nn.setIouThreshold(0.5)
    detection_nn.setNumInferenceThreads(2)
    detection_nn.input.setBlocking(False)

    if hasattr(detection_nn, "setSubtype"):
        detection_nn.setSubtype("yolov8")
        print("Parser: explicit YOLOv8 subtype")
    else:
        print("Parser: no setSubtype() in this DepthAI build; using legacy parser")

    cam_rgb.preview.link(detection_nn.input)
    mono_left.out.link(stereo.left)
    mono_right.out.link(stereo.right)
    stereo.depth.link(detection_nn.inputDepth)

    detection_nn.passthrough.link(xout_rgb.input)
    detection_nn.out.link(xout_nn.input)

    return pipeline


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model",
        choices=("416", "640"),
        default="416",
        help="Which YOLOv8n blob to test.",
    )
    args = parser.parse_args()

    model_size = int(args.model)
    base_dir = Path(__file__).resolve().parent
    blob_path = base_dir / f"blob_v8_{model_size}" / "best.blob"

    if not blob_path.exists():
        raise FileNotFoundError(f"Blob not found: {blob_path}")

    print(f"DepthAI version: {getattr(dai, '__version__', 'unknown')}")
    print(f"Model: {blob_path}")
    print(f"Input size: {model_size}x{model_size}")
    print(f"Labels: {LABELS}")

    pipeline = create_pipeline(model_size, blob_path)

    with dai.Device(pipeline) as device:
        q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        q_det = device.getOutputQueue(name="detections", maxSize=4, blocking=False)

        print("Pipeline started. Press Q to quit.")

        while True:
            frame = q_rgb.get().getCvFrame()
            detections = q_det.get().detections

            h, w = frame.shape[:2]

            for det in detections:
                x1 = max(0, min(w - 1, int(det.xmin * w)))
                y1 = max(0, min(h - 1, int(det.ymin * h)))
                x2 = max(0, min(w - 1, int(det.xmax * w)))
                y2 = max(0, min(h - 1, int(det.ymax * h)))

                label = LABELS[det.label] if 0 <= det.label < len(LABELS) else f"id={det.label}"
                conf = float(det.confidence)

                xyz = det.spatialCoordinates
                text = (
                    f"{label} {conf:.2f}  "
                    f"X:{int(xyz.x)} Y:{int(xyz.y)} Z:{int(xyz.z)} mm"
                )

                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), 2)
                cv2.putText(
                    frame,
                    text,
                    (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )

            cv2.imshow(f"YOLOv8n {model_size} OAK-D Pro test", frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
