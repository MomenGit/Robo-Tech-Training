from abc import ABC, abstractmethod

class IPublisher(ABC):
	"""Interface for publisher classes."""
	
	@abstractmethod
	def publish_msg(self):
		"""Publish a message. Must be implemented by subclasses."""
		pass