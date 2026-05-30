# unitree_im6014_actuator_sdk

SDK for unitree IM6014 actuator.

## 1. Create .clangd

```bash
cd <WorkSpace>
python Clangd_Setup.py -isystem/usr/include/python<Version> -isystem/usr/include/pybind11
```

## 2. Build

```bash
cd <WorkSpace>
sudo chmod +x Build.sh
./Build.sh
```

## 3. Python Interface

```bash
pip install -e .
```
