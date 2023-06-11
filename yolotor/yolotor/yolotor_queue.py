import queue

class YolotorQueue(queue.Queue):
    def __init__(self, maxsize):
        """
        Initializes a YolotorQueue object.

        Args:
            maxsize (int): The maximum size of the queue.
        """
        super().__init__(maxsize)

    def put(self, item, block=True, timeout=None):
        """
        Put an item into the queue.

        If the queue is full, the oldest item will be removed before adding the new item.

        Args:
            item: The item to be added to the queue.
            block (bool, optional): Whether to block if the queue is full. Default is True.
            timeout (float, optional): The maximum time in seconds to wait if block is True. Default is None.

        Raises:
            Full: If the queue is full and block is False.
        """
        if self.full():
            self.get()
        super().put(item, block, timeout)

    def reset(self):
        """
        Reset the queue by removing all elements.
        """
        while not self.empty():
            self.get()
