# ros-dynamixel-docker

## Requirements
- usb2dynamixel
- dynamixel AX12A
- docker (compatible with docker-compose v3)
- docker-compose

## Running
docker-compose up --build

This will start 3 containers. 1 master, 1 publisher, 1 subscriber.

## Results
Normal operation will set the servo at half position.
Position changes by 1% at rate of 10Hz.
When position hits max or min, direction will reverse.
