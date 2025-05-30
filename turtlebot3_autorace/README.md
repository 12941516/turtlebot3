# Managing Autorace Missions
**Environment : Ubuntu 20.04, ROS1 Noetic, NVIDIA Jetson Orin NX 16GB**  
**Topic       : solve autorace missions with turtlebot3**  

## 250523 : Traffic Light, Stop Barrier, RL Detection
### 1. Traffic Light
![trflgt](https://github.com/user-attachments/assets/1a24d28d-5b4b-49bf-b37b-c60d8f9cd8ef)
(result of Traffic Light code)  
```Python
#====================<traffic_light_check>===================#
def traffic_light_check(src):
    src = gaussianBlur(src)
    src = hsv_inrange(src)
    src = morphology(src)
    src = componentsWithStatsFilter(src)
    if np.count_nonzero(src) > 4000:
        rospy.loginfo("RED detected")
        return src, True
    return src, False
```
#### 주요 함수 및 절차
1. gaussianBlur : gaussian blurring을 통한 영상의 노이즈 제거
2. hsv_inrange : 적색 영역만 남기고 전부 False로 필터링
3. morphology : 작은 픽셀 노이즈는 morphology의 dilation 메서드로 제거
4. connectedComponentsWithStatsFilter : 큰 노이즈를 제거하는 필터
5. traffic_light_check : 실제로 구현한 적색 신호 인식 함수

#### 코드 설명  
(전처리 함수는 ros_vision 패키지를 참고할 것)  
gaussianBlur, hsv_inrange, morphology, componentsWithStatsFilter를 사용하여 붉은 영역에 대한 HSV값만을 True로 하는 이진 영상을 생성한다.  
해당 이진 영상에서 numpy의 count_nonzero() 메서드를 적용하여 True인 픽셀의 수가 특정 값 이상인 경우에만 적색등을 인식하였다고 판단할 수 있다.  
  
### 2. Stop Barrier
![stbrr](https://github.com/user-attachments/assets/002de3de-96b7-40b6-a354-31fdb45a7688)
(result of Stop Barrier code)  
```Python
#=======================<stop_barrier>=======================#
def stop_barrier(src):
    ###TODO : Implements codes in this block###
    src = gaussianBlur(src)
    src = hsv_inrange(src)
    src = morphology(src)
    src = componentsWithStatsFilter(src)
    
    coordinates = []
    contours, hierachy = cv2.findContours(src, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    src = cv2.cvtColor(src, cv2.COLOR_GRAY2BGR)
    for contour in contours:
        if cv2.contourArea(contour) < 4000: continue
        m = cv2.moments(contour)
        cx = int(m['m10']/m['m00'])
        cy = int(m['m01']/m['m00'])
        coordinates.append((cx, cy))
        cv2.circle(src, (cx, cy), 5, (255, 100, 100), -1)
    
    slope = []
    inf_count = 0
    if len(coordinates) >= 2:
        for i in range(len(coordinates) - 1):
            dx = coordinates[i][0] - coordinates[i+1][0]
            dy = coordinates[i][1] - coordinates[i+1][1]
            if dx == 0: inf_count += 1
            else: slope.append(abs(dy / dx))

        if inf_count > 0: return src, True
        if slope:
            slope_avg = sum(slope) / len(slope)
            if slope_avg > 1: return src, True
    ###########################################
    return src, False
```
#### 주요 함수 및 절차
1. gaussianBlur : gaussian blurring을 통한 영상의 노이즈 제거
2. hsv_inrange : 적색 영역만 남기고 전부 False로 필터링
3. morphology : 작은 픽셀 노이즈는 morphology의 dilation 메서드로 제거
4. connectedComponentsWithStatsFilter : 큰 노이즈를 제거하는 필터
5. stop_barrier : 실제로 구현한 차단바 여닫힘 인식 함수

#### 코드 설명  
(전처리 함수는 ros_vision 패키지를 참고할 것)  
gaussianBlur, hsv_inrange, morphology, componentsWithStatsFilter를 사용하여 붉은 영역에 대한 HSV값만을 True로 하는 이진 영상을 생성한다.  
해당 이진 영상에 대해 cv2의 moment 메서드를 적용하면 차단바의 붉은 색 영역에 대한 색중심점을 여러개 얻을 수 있다.  
얻어낸 색중심점들의 중심좌표를 사용해 적절한 예외처리와 함께 기울기를 계산하고, 그 값으로 차단바의 여닫힘을 인식한다.  
  
### 3. RL Detection
![right](https://github.com/user-attachments/assets/ccf1c945-388b-479e-af7f-2fe63d8b293b)
![left](https://github.com/user-attachments/assets/407ffce9-4e85-4d9f-a2a8-985aa80011da)
(result of RL Detection code, right and left)  
```Python
#=======================<RL detection>=======================#
def RL_detection(src):
    src = gaussianBlur(src)
    src = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    circles = cv2.HoughCircles(
        src,
        cv2.HOUGH_GRADIENT,
        dp=1,
        minDist=200,
        param1=100,
        param2=60,
        minRadius=50,
        maxRadius=200
    )
    src = cv2.cvtColor(src, cv2.COLOR_GRAY2BGR)
    
    ###TODO : Implements codes in this block###
    gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for (x,y,r) in circles[0,:]:
            cv2.circle(src, (x,y), r, (0,255,0), 2)
            cv2.circle(src, (x,y), 3, (0,0,255), -1)
            try:
                left_check = [int(x-r/3),int(y+r/3)]
                right_check = [int(x+r/3),int(y+r/3)]
                cv2.circle(src, (left_check[0],left_check[1]), 3, (255,50,50), -1)
                cv2.circle(src, (right_check[0],right_check[1]), 3, (255,150,150), -1)        
                if gray[left_check[1],left_check[0]] > gray[right_check[1],right_check[0]]:
                    return src, -1
                elif gray[left_check[1],left_check[0]] < gray[right_check[1],right_check[0]]:
                    return src, 1
            except: rospy.loginfo("\n***out of frame***\n")
    ###########################################
    return src, 0
```
#### 주요 함수 및 절차
1. gaussianBlur : gaussian blurring을 통한 영상의 노이즈 제거
2. RL_detection : 실제로 구현한 좌/우 표지판 검출 함수

#### 코드 설명  
(전처리 함수는 ros_vision 패키지를 참고할 것)  
gaussianBlur로 영상의 노이즈를 제거, cv2.cvtColor() 메서드에 cv2.COLOR_BGR2GRAY 인자를 입력해 단일 채널 영상을 얻는다.  
해당 단일 채널 영상에 cv2.HoughCircles() 메서드를 사용하여 원의 중심좌표, 반지름을 얻는다.  
얻은 원 정보를 사용해 중심점 기준 좌 하단, 우 하단 점의 픽셀 값 밝기를 비교해 좌/우 표지판을 판별한다.  

#### 심화 탐구 : cv2.HoughCircles()의 작동 원리
