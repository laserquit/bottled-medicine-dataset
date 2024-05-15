This project is the supporting code of the paper "Medicine Surface Defect Area Calculation: An Integrated Approach Fusing Instance Segmentation and Perspective Transformation Based on GAM-YOLOv8-SEGMENT"


video folder shows some testing video based on real senario model

This project needs Daheng image camera GalaxySDK and OPENCV to support;
main.cpp need 3 Daheng image industrial cameras to run
## Environment

- **Tensorrt 8.4.3.**
- **Cuda 11.6 Cudnn 8.4.1**
- **onnx 1.12.0**
- use**v8_transform.py**，to sum yolov8n.transd.onnx。

```
python v8_transform.py yolov8n.onnx
```

put**onnx**model into **tensorrt/bin**folder，use**trtexec**released by official to turn to .trt file

```
trtexec --onnx=yolov8n.transd.onnx --saveEngine=yolov8n_fp16.trt --fp16
```

TENSORRT turbo is activated, all the trained models need to turned into .onnx then turned into .trt file
The  .trt provided is based on testing environment, a new trt file needs to be sumed in new environment.

the subsequent code and trained pt model will be uploaded later.
