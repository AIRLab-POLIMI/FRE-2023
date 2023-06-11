from yolotor.yolotor_queue import YolotorQueue
from collections import Counter

class YolotorClassifier:
    def __init__(self, queue_size=10, confidence_threshold=0.30, interval=2):
        """
        Initializes a YolotorClassifier object.

        Args:
            queue_size (int, optional): The maximum size of the internal queue. Default is 20.
            confidence_threshold (float, optional): The confidence threshold for accepting detections. Default is 0.60.
            interval (int, optional): The interval in seconds to reset the queue. Default is 2.
        """
        self.queue_size = queue_size
        self.confidence_threshold = confidence_threshold
        self.interval = interval
        self.first_timestamp = 0
        self.last_timestamp = 0

        self.queue = YolotorQueue(maxsize=queue_size)

    def enqueue(self, confidence, timestamp, label):
        """
        Callback function for detection events.

        Args:
            confidence (float): The confidence score of the detection.
            timestamp (float): The timestamp of the detection event.
            label (str): The label or class of the detection.
        """
        if confidence >= self.confidence_threshold:
            self.last_timestamp = timestamp / 1e9
            if abs(self.last_timestamp - self.first_timestamp) > self.interval:
                self.queue.reset()
                self.first_timestamp = self.last_timestamp
            self.queue.put(label)

    def predict(self):
        """
        Perform a fast prediction by returning the most frequent element in the queue.

        Returns:
            The predicted label or class.
        """
        if self.queue.qsize() >= self.queue_size:
            return Counter(self.queue.queue).most_common(1)[0][0]
        return None