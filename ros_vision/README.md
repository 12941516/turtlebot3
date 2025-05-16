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
![sobel_xy](https://github.com/user-attachments/assets/fc52cd4a-3fbb-4d90-9eca-d09c4e2ce83d)
(Sobel gradient on X, Y directions)  
내용을 입력해주세요  
  
### 2. connectedComponentsWithStatsFilter
![connected](https://github.com/user-attachments/assets/ec6bf9c5-defb-4b02-af71-5afdc23359e3)
(Noise elemenation with cv2.connectedComponentsWithStats())  
내용을 입력해주세요  
  
### 3. Perspective Transformation
![persp](https://github.com/user-attachments/assets/f15c223f-958b-4531-8c96-8bbc754647a8)
(Perspective transformation example)  
  
![persp2](https://github.com/user-attachments/assets/741f6abb-fe44-4e08-b3d6-188044a5e249)
(Perspective transformation function)  
내용을 입력해주세요  
  
### 4. Interpolation with Morphology Dilation
![morph](https://github.com/user-attachments/assets/b3f9bcd8-d003-4aa4-b295-52d681595203)
(Morphology example)  
  
![morph2](https://github.com/user-attachments/assets/c1805527-a4d8-4481-9eef-ffa5a1b2e91d)
(Morphology explanation)  
내용을 입력해주세요  
  
### 5. line centers by sliding window
내용을 입력해주세요  
  
## 250516 : Lane curve Expectation code implementation
**Compute velocity using Curvaturate**  
1. Calculation of curvaturate
2. Compute Twist Velocity(linear_x, angular_z)
![total2](https://github.com/user-attachments/assets/cdb4ae1a-7b55-46eb-9ad8-9443e24a84ca)
(View of running code)  
