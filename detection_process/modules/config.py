
#####################
# YOLO SHEEP DETECT #
#################################################################################

YOLO_WEIGHTS_SHEEP = 'models/Aerial-Auth-Asfenah-DeepBack-PrabsUoL3-9.pt'

SC = 0.1  # confidence threshold for sheep detection
SIOU = 0.5  # IoU threshold for sheep detection (Allow overlap)
SA = False  # agnostic NMS for sheep detection (False better)
SM = 200  # max detections for sheep detection
SV = False  # verbose for sheep detection
SS = False  # stream for sheep detection

#########
# VIDEO #
#################################################################################

# Path to video to make inference upon
VIDEOPATH = 'clips1/DJI_20250617141927_0001_S.MP4'
###########
# MATCHER #
#################################################################################
# Superglue

# max_keypoints. Consider up to this number of keypoints
CMK = 1000

# keypoint_threshold
CKT = 0.05

# nms_radius
CNMS = 2

# sinkhorn_iterations
CSI = 30

# match_threshold
CMT = 0.2


###########
# TRACKER #
#################################################################################
# DeepSORT

#  maxAge
TMA = 20

# numInit
TNI = 0

# nmsMaxOverlap - Max allowed IoU overlap between detections (before NMS removes one).
# Lower to prevent duplicate detections, raise if detections are tight.
TOL = 0.9

# max_cosine_distance
TMC = 0.3

# nn_budget
TNN = 100

# override_track_class
TOTC = True

# embedder
TEM = 'mobilenet'