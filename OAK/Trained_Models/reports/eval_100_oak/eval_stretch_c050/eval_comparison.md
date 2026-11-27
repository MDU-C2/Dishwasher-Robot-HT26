# Unified 6-Class Evaluation Comparison

| Model | Runtime | Preprocess | P | R | F1 | mAP@50 | mAP@50:95 | Empty FP Rate | Latency (ms) |
| :--- | :--- | :--- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `FT-MARC-v5s-416` | oak_blob_fp16 | stretch | 0.8560 | 0.5474 | 0.6518 | 0.6672 | 0.4688 | 0.000 | 95.5 |
| `MARC-v5s-640` | oak_blob_fp16 | stretch | 0.7485 | 0.3221 | 0.3995 | 0.5833 | 0.3995 | 0.000 | 259.0 |
| `YOLO11n-416` | oak_blob_fp16 | stretch | 0.9590 | 0.9477 | 0.9527 | 0.9609 | 0.8574 | 0.125 | 75.6 |
| `YOLO11n-640` | oak_blob_fp16 | stretch | 0.9834 | 0.9758 | 0.9794 | 0.9777 | 0.8794 | 0.250 | 155.0 |
| `YOLOv5s-416` | oak_blob_fp16 | stretch | 0.9667 | 0.9602 | 0.9625 | 0.9696 | 0.8500 | 0.125 | 94.3 |
