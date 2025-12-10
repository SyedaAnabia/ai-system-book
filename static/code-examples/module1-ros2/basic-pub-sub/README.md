# Basic Publisher-Subscriber Example

This example demonstrates the fundamental publish/subscribe communication pattern in ROS 2.

## Files
- `talker.py`: A publisher node that sends "Hello World" messages with an incrementing counter
- `listener.py`: A subscriber node that receives and logs the messages from the publisher

## How to Run

1. Make sure you have ROS 2 Iron installed and sourced:
   ```bash
   source /opt/ros/iron/setup.bash
   ```

2. Run the publisher in one terminal:
   ```bash
   python3 talker.py
   ```

3. In a separate terminal, run the subscriber:
   ```bash
   python3 listener.py
   ```

You should see the publisher sending messages and the subscriber receiving them. The publisher will send a message every 0.5 seconds with an incrementing counter.

## Expected Output

Publisher output:
```
[INFO] [1692027347.502208200] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [1692027348.002441500] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [1692027348.502725400] [minimal_publisher]: Publishing: "Hello World: 2"
```

Subscriber output:
```
[INFO] [1692027347.502301700] [minimal_subscriber]: I heard: "Hello World: 0"
[INFO] [1692027348.002523800] [minimal_subscriber]: I heard: "Hello World: 1"
[INFO] [1692027348.502790200] [minimal_subscriber]: I heard: "Hello World: 2"
```

## Code Explanation

`talker.py` creates a publisher that:
1. Initializes a ROS 2 node named 'minimal_publisher'
2. Creates a publisher for String messages on the 'topic' with a queue size of 10
3. Sets up a timer to publish messages every 0.5 seconds
4. Publishes messages with an incrementing counter

`listener.py` creates a subscriber that:
1. Initializes a ROS 2 node named 'minimal_subscriber'
2. Creates a subscription to the 'topic' for String messages
3. Defines a callback function that logs received messages