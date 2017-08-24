# OptimizerPBA

## 1. Introduction
This is a optimizer plugin base on PBA for GSLAM later then version 2.1.0, which implement parallel bundle adjust with GPU or multi-core CPU.

## 2. Build and Install
### 2.1. Build and install GSLAM

https://github.com/zdzhaoyong/GSLAM

### 2.2. Build and install OptimizerPBA
```
mkdir build;cd build;cmake ..;sudo make install
```
## 3. Usage

```
bool bundleAdjust(GSLAM::BundleGraph& graph)
{
  SPtr<GSLAM::Optimizer> optimizer=GSLAM::Optimizer::create("libgslam_optimizerPBA");
  return optimizer->optimize(graph);
}
```
