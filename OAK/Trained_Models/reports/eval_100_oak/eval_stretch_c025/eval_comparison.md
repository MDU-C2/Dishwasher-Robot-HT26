# Unified 6-Class Evaluation Comparison

| Model | Runtime | Preprocess | P | R | F1 | mAP@50 | mAP@50:95 | Empty FP Rate | Latency (ms) |
| :--- | :--- | :--- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `FT-MARC-v5s-416` | oak_blob_fp16 | stretch | 0.7952 | 0.6052 | 0.6744 | 0.6672 | 0.4688 | 0.000 | 95.5 |
| `MARC-v5s-640` | oak_blob_fp16 | stretch | 0.7613 | 0.4701 | 0.5296 | 0.5833 | 0.3995 | 0.000 | 259.0 |
| `YOLO11n-416` | oak_blob_fp16 | stretch | 0.9592 | 0.9530 | 0.9555 | 0.9609 | 0.8574 | 0.125 | 75.6 |
| `YOLO11n-640` | oak_blob_fp16 | stretch | 0.9834 | 0.9853 | 0.9842 | 0.9777 | 0.8794 | 0.250 | 155.0 |
| `YOLOv5s-416` | oak_blob_fp16 | stretch | 0.9627 | 0.9615 | 0.9608 | 0.9696 | 0.8500 | 0.125 | 94.3 |
