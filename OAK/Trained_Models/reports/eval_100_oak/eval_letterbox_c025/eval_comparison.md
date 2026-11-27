# Unified 6-Class Evaluation Comparison

| Model | Runtime | Preprocess | P | R | F1 | mAP@50 | mAP@50:95 | Empty FP Rate | Latency (ms) |
| :--- | :--- | :--- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `FT-MARC-v5s-416` | oak_blob_fp16 | letterbox | 0.9040 | 0.7782 | 0.8298 | 0.8022 | 0.6250 | 0.125 | 94.6 |
| `MARC-v5s-640` | oak_blob_fp16 | letterbox | 0.7163 | 0.3793 | 0.4303 | 0.4253 | 0.2711 | 0.000 | 257.4 |
| `YOLO11n-416` | oak_blob_fp16 | letterbox | 0.9399 | 0.8109 | 0.8670 | 0.8680 | 0.7240 | 0.000 | 75.9 |
| `YOLO11n-640` | oak_blob_fp16 | letterbox | 0.9502 | 0.8573 | 0.9009 | 0.9228 | 0.7688 | 0.000 | 154.9 |
| `YOLOv5s-416` | oak_blob_fp16 | letterbox | 0.9172 | 0.7829 | 0.8414 | 0.8191 | 0.6182 | 0.000 | 94.9 |
