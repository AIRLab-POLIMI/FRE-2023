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

	print("[OAK-D/INFO] loading model config...")
	configPath = Path(config_path)
	model_config = load_config(configPath)
	nnConfig = model_config.get("nn_config", {})

	print("[OAK-D/INFO] extracting metadata from model config...")
	metadata = nnConfig.get("NN_specific_metadata", {})
	classes = metadata.get("classes", {})
	coordinates = metadata.get("coordinates", {})
	anchors = metadata.get("anchors", {})
	anchorMasks = metadata.get("anchor_masks", {})
	iouThreshold = metadata.get("iou_threshold", {})
	confidenceThreshold = metadata.get("confidence_threshold", {})
	print(metadata)

	print("[OAK-D/INFO] initializing OAK modules (camera, stereo, network)...")
	camRgb = pipeline.create(dai.node.ColorCamera)
	detectionNetwork = pipeline.create(dai.node.YoloDetectionNetwork)
	monoLeft = pipeline.create(dai.node.MonoCamera)
	monoRight = pipeline.create(dai.node.MonoCamera)
	depth = pipeline.create(dai.node.StereoDepth)

	print("[OAK-D/INFO] configuring modules outputs...")
	# camera
	xoutRgb = pipeline.create(dai.node.XLinkOut)
	xoutRgb.setStreamName("rgb")
	# depth
	xoutDep = pipeline.create(dai.node.XLinkOut)
	xoutDep.setStreamName("depth")
	# network
	nnOut = pipeline.create(dai.node.XLinkOut)
	nnOut.setStreamName("nn")

	print("[OAK-D/INFO](camera) setting module properties...")
	camRgb.setPreviewSize(camera_dim)
	camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
	camRgb.setInterleaved(False)
	camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
	camRgb.setFps(40)

	print("[OAK-D/INFO](network) setting module properties...")
	detectionNetwork.setConfidenceThreshold(confidenceThreshold)
	detectionNetwork.setNumClasses(classes)
	detectionNetwork.setCoordinateSize(coordinates)
	detectionNetwork.setAnchors(anchors)
	detectionNetwork.setAnchorMasks(anchorMasks)
	detectionNetwork.setIouThreshold(iouThreshold)
	detectionNetwork.setBlobPath(model_path)
	detectionNetwork.setNumInferenceThreads(2)
	detectionNetwork.input.setBlocking(False)

	print("[OAK-D/INFO](stereo) setting module properties...")
	# Closer-in minimum depth, disparity range is doubled (from 95 to 190):
	extended_disparity = False
	# Better accuracy for longer distance, fractional disparity 32-levels:
	subpixel = False
	# Better handling for occlusions:
	lr_check = True

	# Properties
	monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
	monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
	monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
	monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

	# Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
	depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)

	# Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
	depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
	depth.setLeftRightCheck(lr_check)
	depth.setExtendedDisparity(extended_disparity)
	depth.setSubpixel(subpixel)

	print("[OAK-D/INFO] creating links among modules...")
	camRgb.preview.link(detectionNetwork.input)
	detectionNetwork.passthrough.link(xoutRgb.input)
	detectionNetwork.out.link(nnOut.input)
	monoLeft.out.link(depth.left)
	monoRight.out.link(depth.right)
	depth.disparity.link(xoutDep.input)

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
