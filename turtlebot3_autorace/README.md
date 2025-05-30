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
**주요 함수 및 절차**
1. gaussianBlur : gaussian blurring을 통한 영상의 노이즈 제거
2. hsv_inrange : 적색 영역만 남기고 전부 False로 필터링
3. morphology : 작은 픽셀 노이즈는 morphology의 dilation 메서드로 제거
4. connectedComponentsWithStatsFilter : 큰 노이즈를 제거하는 필터
5. traffic_light_check : 실제로 구현한 적색 신호 인식 함수

**코드 설명**  
gaussianBlur, hsv_inrange, morphology, componentsWithStatsFilter를 사용하여 붉은 영역에 대한 HSV값만을 True로 하는 이진 영상을 생성한다.  
해당 이진 영상에서 numpy의 count_nonzero() 메서드를 적용하여 True인 픽셀의 수가 특정 값 이상인 경우에만 적색등을 인식하였다고 판단할 수 있다.  
  
### 2. connectedComponentsWithStatsFilter
![image](https://github.com/user-attachments/assets/76476c7e-b313-4b2e-bca9-0a4e2f5b6e0b)
(Noise elemenation with cv2.connectedComponentsWithStats())  
```Python
#=================<ComponentsWithStatsFilter>================#
def componentsWithStatsFilter(src):
    min_area = 50
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(src, connectivity=8)
    valid_labels = np.where(stats[1:, cv2.CC_STAT_AREA] >= min_area)[0] + 1
    mask = np.isin(labels, valid_labels)
    filtered = (mask * 255).astype(np.uint8)
    return filtered
```
이진 영상의 연결 객체에 대하여, min_area보다 작은 영역은 0으로 값을 바꿔 노이즈를 제거한다.  
cv2.connectedComponentsWithStats() 함수는 인자로 src, connectivity를 받는다.  
src는 이진 영상이며, connectivity는 4 또는 8의 값으로 연결 방향을 결정한다.  
cv2.connectedComponentsWithStats() 함수의 반환 값으로 라벨 수, 라벨 번호, 각 객체의 면적 등이 포함된 통계 정보를 사용한다.  
면적이 min_area보다 크다면 valid_labels 리스트에 저장하며, 여기에는 np.where() 메서드를 사용한다.  
np.isin(labels, valid_labels)를 통해 valid_labels에 저장된 라벨을 1로 하여 mask를 만들고, mask 행렬에 255를 곱하여 유효 객체만 흰색으로 보정한다.  
  
### 3. Perspective Transformation
![image](https://github.com/user-attachments/assets/277efefb-1531-4b8f-8da6-d8d91847138b)
(Perspective transformation function)  
내용을 입력해주세요  
  
### 4. Interpolation with Morphology Dilation
![image](https://github.com/user-attachments/assets/0e3619f8-c40c-4f3e-8792-a4ad761c9334)
(Morphology example)   
내용을 입력해주세요  
  
### 5. line centers by sliding window
![image](https://github.com/user-attachments/assets/f639567f-acee-45f4-834f-8cf74b8a8743)
![image](https://github.com/user-attachments/assets/64fbe092-69a5-4f85-bd4e-d83b5279b374)
![image](https://github.com/user-attachments/assets/6cf13694-ce9b-4a9d-8569-a417fbff3d62)
![image](https://github.com/user-attachments/assets/167a9194-e908-4753-b7ba-6da09a337bb6)
![image](https://github.com/user-attachments/assets/4a1dc778-8b58-4cae-bf3b-08b8d5a879bc)
(Sliding Windows Procedures)  
내용을 입력해주세요  
  
## 250516 : Lane curve Expectation code implementation
**Compute velocity using Curvaturate**  
1. Calculation of curvaturate
2. Compute Twist Velocity(linear_x, angular_z)
  
![total2](https://github.com/user-attachments/assets/cdb4ae1a-7b55-46eb-9ad8-9443e24a84ca)
(View of running code)  
