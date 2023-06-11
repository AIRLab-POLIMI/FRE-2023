import depthai as dai
import json
import numpy as np
import cv2

from pathlib import Path


def load_config(config_path):
	"""
	JSON parser.

	Parameters
	----------
		- config_path:
			JSON file path.
	"""
	with config_path.open() as f:
		config = json.load(f)

	return config


def create_camera_pipeline(config_path, model_path, camera_dim):
	"""
	Create the inference pipeline for the OAK camera.

	Parameters
	----------
		- config_path:
			Model configuration file path.

		- model_path:
			Model blob file path.

		- camera_dim:
			Input image size; assuming it is squared, then just one dimension.
	"""
	pipeline = dai.Pipeline()

	print("[INFO] loading model config...")
	configPath = Path(config_path)
	model_config = load_config(configPath)
	nnConfig = model_config.get("nn_config", {})

	print("[INFO] extracting metadata from model config...")
	metadata = nnConfig.get("NN_specific_metadata", {})
	classes = metadata.get("classes", {})
	coordinates = metadata.get("coordinates", {})
	anchors = metadata.get("anchors", {})
	anchorMasks = metadata.get("anchor_masks", {})
	iouThreshold = metadata.get("iou_threshold", {})
	confidenceThreshold = metadata.get("confidence_threshold", {})
	print(metadata)

	print("[INFO] configuring source and outputs...")
	camRgb = pipeline.create(dai.node.ColorCamera)
	detectionNetwork = pipeline.create(dai.node.YoloDetectionNetwork)
	xoutRgb = pipeline.create(dai.node.XLinkOut)
	nnOut = pipeline.create(dai.node.XLinkOut)

	print("[INFO] setting stream names for queues...")
	xoutRgb.setStreamName("rgb")
	nnOut.setStreamName("nn")

	print("[INFO] setting camera properties...")
	camRgb.setPreviewSize(camera_dim)
	camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
	camRgb.setInterleaved(False)
	camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
	camRgb.setFps(40)

	print("[INFO] setting YOLO network properties...")
	detectionNetwork.setConfidenceThreshold(confidenceThreshold)
	detectionNetwork.setNumClasses(classes)
	detectionNetwork.setCoordinateSize(coordinates)
	detectionNetwork.setAnchors(anchors)
	detectionNetwork.setAnchorMasks(anchorMasks)
	detectionNetwork.setIouThreshold(iouThreshold)
	detectionNetwork.setBlobPath(model_path)
	detectionNetwork.setNumInferenceThreads(2)
	detectionNetwork.input.setBlocking(False)

	print("[INFO] creating links...")
	camRgb.preview.link(detectionNetwork.input)
	detectionNetwork.passthrough.link(xoutRgb.input)
	detectionNetwork.out.link(nnOut.input)

	return pipeline


def annotateFrame(frame, detection, labels):
	"""
	Annotation tool for debugging purpouse in object detection.

	Parameters
	----------
		- frame:
			OpenCV frame.

		- detection:
			Only one detection computed by YOLO.

		- labels:
			Set of labels recognized by YOLO.
	"""
	# internal method used only here
	def frameNorm(frame, bbox):
		normVals = np.full(len(bbox), frame.shape[0])
		normVals[::2] = frame.shape[1]
		return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

	# labels colors
	colors = [(0, 255, 0), (0, 0, 255)]

	# bounding boxes display
	bbox = frameNorm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
	cv2.putText(frame, labels[detection.label - 1], (bbox[0] + 10, bbox[1] + 25),
				cv2.FONT_HERSHEY_SIMPLEX, 1, colors[detection.label - 1], 2)
	cv2.putText(frame, f"{int(detection.confidence * 100)}%", (bbox[0] + 10, bbox[1] + 60),
				cv2.FONT_HERSHEY_SIMPLEX, 1, colors[detection.label - 1], 2)
	cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), colors[detection.label - 1], 2)

	return frame
