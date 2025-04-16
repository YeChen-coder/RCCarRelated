# Lane Keep Assist for McMaster RC Car Project

## Demo

[![Demo](https://img.youtube.com/vi/nsVGrZQ1r80/0.jpg)](https://www.youtube.com/watch?v=nsVGrZQ1r80&list=PL-WltkCZLf4pbiq5nPD7VYbRuJJEh1OGw&index=1)

## Everything Explained

Check the [SEP742 - Group 1 - Lane Keep on AC Car.pdf](SEP742%20-%20Group%201%20-%20Lane%20Keep%20on%20AC%20Car.pdf) file for more information. 

## Development

### Environment

Step 1: Create a Python virtual environment:
```bash
python3 -m venv path/to/venv 
```

Step 2: Activate the virtual environment:
```bash
source path/to/venv/bin/activate
```

Step 3: Run the following command to install the dependencies:
```bash
cd path/to/RCCarRelated
pip install -r requirements.txt
```

### Run

#### AutoRCCar Control Script  

This script is designed to control an RC car with different modes of operation, including testing, autonomous driving, and debugging. It allows customization of parameters like speed, frame rate, PID controller weights, and steering control.  

---  

#### Usage  

```bash  
python auto_drive.py <mode> [--speed <float>] [--fps <int>] [--pid <tuple>] [--steer_weight <float>]  
```

#### Example

The debug mode, car will run in steps when user press keyboard, until user press 'q':
```
cd path/to/RCCarRelated/utils

python3 auto_drive.py debug --speed 38 --fps 2 --pid "(0.5, 0.1, 0.1)" --steer_weight 1.0
```

The auto mode, car will run drived by camera, until user press 'q':
```
cd path/to/RCCarRelated/utils

python3 auto_drive.py auto --speed 40 --fps 10 --steer_weight 0.5
```
