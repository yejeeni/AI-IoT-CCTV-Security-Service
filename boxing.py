import cv2
import time
import numpy as np
import matplotlib.pyplot as plt
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
from datetime import datetime
import os

weightsFile = './pose_iter_584000.caffemodel'
protoFile = './pose_deploy.prototxt'

# YOLO 설정
weights_path = "./yolov3.weights"
config_path = "./yolov3.cfg"
class_names_path = "./coco.names"

nPoints = 25

# 이 변수는 키포인트의 인덱스에 대한 이름을 나타내는 리스트입니다. 예를 들어, keypointsMapping[0]은 "Nose" 키포인트의 인덱스를 나타내고,
# keypointsMapping[1]은 "Neck" 키포인트의 인덱스를 나타냅니다. 이것은 인체의 다양한 부위를 나타내는 키포인트들을 구분하기 위해 사용됩니다.
keypointsMapping = ["Nose","Neck","RShoulder", "RElbow", "RWrist", "LShoulder", "LElbow",
                    "LWrist", "MidHip", "RHip","RKnee", "RAnkle", "LHip", "LKnee",
                    "LAnkle", "REye", "LEye", "REar", "LEar", "LBigToe", "LSmallToe",
                     "LHeel",  "RBigToe", "RSmallToe", "RHeel"]

# 이 변수는 서로 연결된 키포인트 쌍을 나타내는 리스트입니다. 예를 들어, [1,2]는 "Neck"와 "RShoulder" 키포인트 사이의 연결을 나타내며,
# [1,5]는 "Neck"와 "LShoulder" 키포인트 사이의 연결을 나타냅니다. 이 연결 정보는 인체의 뼈대를 형성하는 키포인트들을 연결하기 위해 사용됩니다.
POSE_PAIRS = [[1,2], [1,5], [2,3], [3,4], [5,6], [6,7],     
              [1,8], [8,9], [9,10], [10,11], [8,12], [12,13], [13,14], 
              [11,24], [11,22], [22,23], [14,21],[14,19],[19,20],  
              [1,0], [0,15], [15,17], [0,16], [16,18],
              [2,17], [5,18]]

# 이 변수는 PAF(Part Affinity Fields) 연결 정보를 다루기 위한 리스트입니다. PAF는 키포인트 사이의 관계를 나타내는데 사용되며, PAF의 두 부분이 어떤 키포인트를 연결하는지를 나타냅니다.
# 예를 들어, [40,41]은 "Nose"와 "Neck" 키포인트를 연결하는 PAF 정보를 나타냅니다. 이 정보는 키포인트 사이의 관계를 추론하는데 사용됩니다.
mapIdx = [[40,41],[48,49],[42,43],[44,45],[50,51],[52,53],
          [26,27],[32,33],[28,29],[30,31],[34,35],[36,37],
          [38,39],[76,77],[72,73],[74,75],[70,71],[66,67],
          [68,69],[56,57],[58,59],[62,63],[60,61],[64,65],
          [46,47],[54,55]]

# 색상 정보
colors = [ [0,100,255], [0,100,255], [0,255,255], [0,100,255], [0,255,255], [0,100,255],
         [0,255,0], [255,200,100], [255,0,255], [0,255,0], [255,200,100], [255,0,255],
         [0,0,255], [255,0,0], [200,200,0], [255,0,0], [125,200,125], [125,200,0],
         [200,200,200],[200,100,200],[200,200,0],[0,200,0],[200,0,255],[0,250,125],
         [0,200,0],[0,120,200]]

# 확률 맵으로부터 키포인트를 추출하는 함수입니다. Gaussian Blur와 이진화를 사용하여 윤곽을 추출하고, 추출한 윤곽 내에서 지역 최댓값인 키포인트를 찾음
def getKeypoints(probMap, threshold=0.1):

    # 확률 맵(probMap)을 3x3 크기의 가우시안 블러(Gaussian Blur)로 노이즈를 줄입니다.
    mapSmooth = cv2.GaussianBlur(probMap,(3,3),0,0)

    mapMask = np.uint8(mapSmooth>threshold)
    keypoints = []
    #find the blobs
    contours, _ = cv2.findContours(mapMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
   
    #for each blob find the maxima
    for cnt in contours:
       blobMask = np.zeros(mapMask.shape)
       blobMask = cv2.fillConvexPoly(blobMask, cnt, 1)
       maskedProbMap = mapSmooth * blobMask
       #maxVal = 있을 확률
       #maxLoc = x,y좌표
       _, maxVal, _, maxLoc = cv2.minMaxLoc(maskedProbMap)
       keypoints.append(maxLoc + (probMap[maxLoc[1], maxLoc[0]],))
    return keypoints

# PAF (Part Affinity Fields) 점수를 기반으로 유효한 연결과 무효한 연결을 찾는 함수입니다. 모든 키포인트 쌍에 대해 PAF 점수를 계산하고, 일정 임계값을 넘는 연결을 유효한 연결로 간주
def getValidPairs(output):
    valid_pairs = []
    invalid_pairs = []
    n_interp_samples = 10 # 벡터를 나눌 수
    paf_score_th = 0.1
    conf_th = 0.7
    # loop for every POSE_PAIR
    for k in range(len(mapIdx)):
        # A->B constitute a limb
        pafA = output[0, mapIdx[k][0], :, :]
        pafB = output[0, mapIdx[k][1], :, :]
        pafA = cv2.resize(pafA, (frameWidth, frameHeight))
        pafB = cv2.resize(pafB, (frameWidth, frameHeight))

        # Find the keypoints for the first and second limb
        candA = detected_keypoints[POSE_PAIRS[k][0]]
        candB = detected_keypoints[POSE_PAIRS[k][1]]
        nA = len(candA)
        nB = len(candB)

        if(nA != 0 and nB != 0):
            valid_pair = np.zeros((0,3))
            for i in range(nA):
                max_j=-1
                maxScore = -1
                found = 0
                for j in range(nB):
                    # Find d_ij
                    d_ij = np.subtract(candB[j][:2], candA[i][:2])
                    norm = np.linalg.norm(d_ij)
                    if norm:
                        d_ij = d_ij / norm
                    else:
                        continue
                    # Find p(u)
                    interp_coord = list(zip(np.linspace(candA[i][0], candB[j][0], num=n_interp_samples),
                                            np.linspace(candA[i][1], candB[j][1], num=n_interp_samples)))
                    # Find L(p(u))
                    paf_interp = []
                    for k in range(len(interp_coord)):
                        paf_interp.append([pafA[int(round(interp_coord[k][1])), int(round(interp_coord[k][0]))],
                                           pafB[int(round(interp_coord[k][1])), int(round(interp_coord[k][0]))]]) 
                    # Find E
                    paf_scores = np.dot(paf_interp, d_ij)
                    avg_paf_score = sum(paf_scores)/len(paf_scores)
                    
                    # Check if the connection is valid
                    # If the fraction of interpolated vectors aligned with PAF is higher then threshold -> Valid Pair  
                    if (len(np.where(paf_scores > paf_score_th)[0]) / n_interp_samples) > conf_th:
                        if avg_paf_score > maxScore:
                            max_j = j
                            maxScore = avg_paf_score
                            found = 1
                # Append the connection to the list
                if found:            
                    valid_pair = np.append(valid_pair, [[candA[i][3], candB[max_j][3], maxScore]], axis=0)
                    
            # Append the detected connections to the global list
            valid_pairs.append(valid_pair)
        else: # If no keypoints are detected
            #print("No Connection : k = {}".format(k)) # 인식 못한 포인트 출력
            invalid_pairs.append(k)
            valid_pairs.append([])

    return valid_pairs, invalid_pairs


# 유효한 연결 정보를 사용하여 각 사람별로 키포인트를 할당하는 함수입니다. 유효한 연결에 따라 각 사람에게 키포인트를 할당하고, 각 사람의 키포인트들을 저장
def getPersonwiseKeypoints(valid_pairs, invalid_pairs):
    # the last number in each row is the overall score 
    personwiseKeypoints = -1 * np.ones((0, 26))

    for k in range(len(mapIdx)):
        if k not in invalid_pairs:
            partAs = valid_pairs[k][:,0]
            partBs = valid_pairs[k][:,1]
            indexA, indexB = np.array(POSE_PAIRS[k])

            for i in range(len(valid_pairs[k])): 
                found = 0
                person_idx = -1
                for j in range(len(personwiseKeypoints)):
                    if personwiseKeypoints[j][indexA] == partAs[i]:
                        person_idx = j
                        found = 1
                        break

                if found:
                    personwiseKeypoints[person_idx][indexB] = partBs[i]
                    personwiseKeypoints[person_idx][-1] += keypoints_list[partBs[i].astype(int), 2] + valid_pairs[k][i][2]

                # if find no partA in the subset, create a new subset
                elif not found and k < 24:
                    row = -1 * np.ones(26)
                    row[indexA] = partAs[i]
                    row[indexB] = partBs[i]
                    # add the keypoint_scores for the two keypoints and the paf_score
                    row[-1] = sum(keypoints_list[valid_pairs[k][i,:2].astype(int), 2]) + valid_pairs[k][i][2]
                    personwiseKeypoints = np.vstack([personwiseKeypoints, row])

    return personwiseKeypoints

# 사람이 누워있는지 판단하는 함수
def is_laying_down(keypoints, det):
    for i in range(len(keypoints[keypointsMapping.index("Neck")])):
        y_diff_neck_left_wrist = None
        y_diff_neck_right_wrist = None
        y_diff_neck_mid_hip = None
        y_diff_neck_left_ankle = None
        y_diff_neck_right_ankle = None

        for d in det:
            if d[0] <= keypoints[keypointsMapping.index("Neck")][i][0] <= d[0] + d[2] and d[1] < keypoints[keypointsMapping.index("Neck")][i][1] < d[1] + d[3]:
                point = d # 목 위치가 바운딩 안에 있으면 해당 바운딩을 저장
                break

        for j in ["Neck", "LWrist", "RWrist", "MidHip", "LAnkle", "RAnkle"]:
            try:
                # 모든 위치가 바운딩 범위 안에 포함이 되면 해당 위치로 받을 수 있도록함
                if(point[0] <= keypoints[keypointsMapping.index(j)][i][0] <= point[0] + point[2] and point[1] <= keypoints[keypointsMapping.index(j)][i][1] <= point[1] + point[3]):
                    if j == "Neck":
                        neck = keypoints[keypointsMapping.index("Neck")][i]
                        neck_y = neck[1]
                        print("neck의 y 값", neck_y)

                    if j == "LWrist":
                        left_wrist = keypoints[keypointsMapping.index("LWrist")][i]
                        left_wrist_y = left_wrist[1]
                        y_diff_neck_left_wrist = abs(neck_y - left_wrist_y)
                        print("left_wrist의 y 값", left_wrist_y)

                    if j == "RWrist":
                        right_wrist = keypoints[keypointsMapping.index("RWrist")][i]
                        right_wrist_y = right_wrist[1]
                        y_diff_neck_right_wrist = abs(neck_y - right_wrist_y)
                        print("right_wrist의 y 값", right_wrist_y)

                    if j == "MidHip":
                        mid_hip = keypoints[keypointsMapping.index("MidHip")][i]
                        mid_hip_y = mid_hip[1]
                        y_diff_neck_mid_hip = abs(neck_y - mid_hip_y)
                        print("mid_hip의 y 값", mid_hip_y)

                    if j == "LAnkle":
                        left_ankle = keypoints[keypointsMapping.index("LAnkle")][i]
                        left_ankle_y = left_ankle[1]
                        y_diff_neck_left_ankle = abs(neck_y - left_ankle_y)
                        print("left_ankle의 y 값", left_ankle_y)

                    if j == "RAnkle":
                        right_ankle = keypoints[keypointsMapping.index("RAnkle")][i]
                        right_ankle_y = right_ankle[1]
                        y_diff_neck_right_ankle = abs(neck_y - right_ankle_y)
                        print("right_ankle의 y 값", right_ankle_y)

            except:
                pass

        #print(f"tolerance 설정을 위한 결과보기 : {y_diff_neck_left_wrist, y_diff_neck_right_wrist, y_diff_neck_mid_hip, y_diff_neck_left_ankle, y_diff_neck_right_ankle}")  
        
        tolerance = 150

        for decision in [y_diff_neck_left_wrist, y_diff_neck_right_wrist, y_diff_neck_mid_hip, y_diff_neck_left_ankle, y_diff_neck_right_ankle]:
            if decision is not None and decision >= tolerance: # 목과의 차이가 정의되어있는데 그것이 tolerance를 넘으면 break
                break
        else: # for문을 전부 돌게 되면 True를 반환
            return True

    return False

def output(image1, det, protoFile, weightsFile):
    global frameWidth, frameHeight
    global detected_keypoints
    global keypoints_list

    frameWidth = image1.shape[1]
    frameHeight = image1.shape[0]

    t = time.time()

    # 네트워크 불러오기
    net = cv2.dnn.readNetFromCaffe(protoFile, weightsFile)

    # Fix the input Height and get the width according to the Aspect Ratio
    inHeight = 368
    inWidth = int((inHeight/frameHeight)*frameWidth)

    # 네트워크에 넣기 위한 전처리
    inpBlob = cv2.dnn.blobFromImage(image1, 1.0 / 255, (inWidth, inHeight), (0, 0, 0), swapRB=False, crop=False)

    # 전처리된 blob 네트워크에 입력
    net.setInput(inpBlob)

    # 결과 받아오기
    output = net.forward()

    detected_keypoints = []
    keypoints_list = np.zeros((0,3))
    keypoint_id = 0
    threshold = 0.1

    # 인식한 신체부위를 리스트에 담음
    for part in range(nPoints):
        probMap = output[0,part,:,:]
        probMap = cv2.resize(probMap, (image1.shape[1], image1.shape[0]))
        keypoints = getKeypoints(probMap, threshold)
        keypoints_with_id = []
        for i in range(len(keypoints)):
            keypoints_with_id.append(keypoints[i] + (keypoint_id,))
            keypoints_list = np.vstack([keypoints_list, keypoints[i]])
            keypoint_id += 1

        detected_keypoints.append(keypoints_with_id)

    # 인식한 신체부위 빨간점 찍기
    frameClone = image1.copy()
    for i in range(nPoints):
        for j in range(len(detected_keypoints[i])):
            cv2.circle(frameClone, detected_keypoints[i][j][0:2], 3, [0,0,255], -1, cv2.LINE_AA)

    valid_pairs, invalid_pairs = getValidPairs(output)

    personwiseKeypoints = getPersonwiseKeypoints(valid_pairs, invalid_pairs)
    
    # 모든 신체 부위를 연결시켜주는 코드
    for i in range(24):
        for n in range(len(personwiseKeypoints)):
            index = personwiseKeypoints[n][np.array(POSE_PAIRS[i])]
            if -1 in index:
                continue
            B = np.int32(keypoints_list[index.astype(int), 0])
            A = np.int32(keypoints_list[index.astype(int), 1])
            cv2.line(frameClone, (B[0], A[0]), (B[1], A[1]), colors[i], 3, cv2.LINE_AA)

    # 검출된 키포인트 사용하여 누워있는지 판단
    laying_down_found = False  # 초기값: 누워있는 사람 없음

    if is_laying_down(detected_keypoints, det):
        laying_down_found = True

    if laying_down_found:
        print("위급 상황")
        plt.figure(figsize=[10,8])
        plt.imshow(frameClone[:,:,[2,1,0]])
        plt.show()
        ref.update({'siren':True})
    else:
        print("평상시")
        ref.update({'siren':False})

def detected(image_path, weights_path, config_path, class_names_path):
    # Yolo 로드
    net = cv2.dnn.readNet(weights_path, config_path)
    classes = []
    with open(class_names_path, "r") as f:
        classes = [line.strip() for line in f.readlines()]
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i-1] for i in net.getUnconnectedOutLayers()]
    colors = np.random.uniform(0, 255, size=(len(classes), 3))

    # 이미지 가져오기
    img = cv2.imread(image_path)
    img = cv2.resize(img, None, fx=1.5, fy=1.5)
    height, width, channels = img.shape

    # Detecting objects
    blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)
                    
    # 정보를 화면에 표시
    class_ids = []
    confidences = []
    boxes = []
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            if class_id == 0: # 사람이라면
                confidence = scores[class_id]
                if confidence > 0.5:
                    # Object detected
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    # 좌표
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
        
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

    font = cv2.FONT_HERSHEY_PLAIN
    for i in range(len(boxes)):
        if i in indexes:
            x, y, w, h = boxes[i]
            label = str(classes[class_ids[i]])
            color = colors[i]
            cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
            cv2.putText(img, label, (x, y), font, 2, color, 2)
    return img, boxes


# 데이터베이스 연결 부분
cred = credentials.Certificate("./cctv-project-609d3-firebase-adminsdk-84ol7-61e6097f79.json")
firebase_admin.initialize_app(cred,{
    'databaseURL' : 'https://cctv-project-609d3-default-rtdb.firebaseio.com/'
})

ref = db.reference('capture')

# 7초마다 사진을 받아와서 쓰러진 사람 검출하는 부분
while True:
    image_path = "\\\\192.168.137.20\\pi\\HF\\test.jpg"
    image_path = "C:\\image1\\2.jpg"
    if os.path.exists(image_path):
        img, det = detected(image_path=image_path, weights_path=weights_path, config_path=config_path, class_names_path=class_names_path)
        output(image1=img, det=det, protoFile=protoFile, weightsFile=weightsFile)
        # os.remove(image_path)
    else:
        print("미인식")

    # 사진을 찍을 때마다 해당 시간을 데이터베이스에 전송
    now = datetime.now()
    current_time = now.strftime("%Y-%m-%d %H:%M:%S")
    ref.update({'time':current_time})
    time.sleep(5)