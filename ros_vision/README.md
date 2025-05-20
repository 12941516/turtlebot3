# Lane Detection with Sliding Windows
**Environment : Ubuntu 20.04, ROS1 Noetic, NVIDIA Jetson Orin NX 16GB**  
**Topic       : line detecting by sliding window**  

## 250513 : Lane detection code implementation
**line detecting procedure**  
1. Preprocessing Video(Edge detection, Noise elemenation, Perspective transformation, Morphology interpolation)
2. Lane detecting with Sliding Windows
  
![total](https://github.com/user-attachments/assets/29756abc-c632-446a-913b-566bfb14c6b5)
(View of running code)  

### 1. Edge detection with sobel_XY_gradients
![image](https://github.com/user-attachments/assets/611f0631-1bbb-40a8-971c-6306aebb80cf)
(Sobel gradient on X, Y directions)  
내용을 입력해주세요  
  
### 2. connectedComponentsWithStatsFilter
![image](https://github.com/user-attachments/assets/76476c7e-b313-4b2e-bca9-0a4e2f5b6e0b)
(Noise elemenation with cv2.connectedComponentsWithStats())  
내용을 입력해주세요  
  
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

내용을 입력해주세요  
  
## 250516 : Lane curve Expectation code implementation
**Compute velocity using Curvaturate**  
1. Calculation of curvaturate
2. Compute Twist Velocity(linear_x, angular_z)
  
![total2](https://github.com/user-attachments/assets/cdb4ae1a-7b55-46eb-9ad8-9443e24a84ca)
(View of running code)  
