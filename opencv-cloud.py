import cv2
import numpy as np
import random

# 视频尺寸
video_size = (960, 540)

# 创建背景
def create_sky(size):
    sky = np.zeros((size[1], size[0], 3), dtype=np.uint8)
    sky[:, :, :] = [245, 186, 100]
    return sky

# 创建云朵
def create_cloud(size, center, axes, angle):
    cloud = np.zeros((size[1], size[0], 4), dtype=np.uint8)
    cv2.ellipse(cloud, center=center, axes=axes, angle=angle,
                startAngle=0, endAngle=360, color=(255, 255, 255, 255),
                thickness=-1)
    return cloud

# 创建视频写入器
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('output.mp4', fourcc, 60.0, video_size)

# 初始化云朵
clouds = []
for _ in range(5):
    cloud_size = (np.random.randint(200, 300), np.random.randint(200, 300))
    cloud_center = (-cloud_size[0]//2+200, np.random.randint(cloud_size[1]//2, video_size[1]-cloud_size[1]//2))
    cloud_axes = (np.random.randint(cloud_size[0]//4, cloud_size[0]//2),
                  np.random.randint(cloud_size[1]//4, cloud_size[1]//2))
    cloud_angle = np.random.randint(0, 360)
    cloud_speed = random.uniform(1, 3)  # 设置云朵的水平速度
    
    cloud = create_cloud(cloud_size, cloud_center, cloud_axes, cloud_angle)
    # 由于我们创建云朵时使用了全不透明的颜色，这里需要调整Alpha通道以避免闪烁
    # cloud[:, :, 3] = 255  # 或者使用一个合适的透明度值，例如 180
    clouds.append({'cloud': cloud, 'center': cloud_center, 'speed': cloud_speed})

for t in range(0, 600):
    sky = create_sky(video_size)
    
    for cloud_data in clouds:
        cloud = cloud_data['cloud']
        cloud_center = cloud_data['center']
        cloud_speed = cloud_data['speed']
        
        # 更新云朵位置
        new_center = (
            int(cloud_center[0] + cloud_speed),
            cloud_center[1]
        )
        
        # 当云朵完全移出画面时，将其重置到左侧
        if new_center[0] > video_size[0]:
            new_center = (-cloud.shape[1]//2, cloud_center[1])
        
        # 移除透明度变化和高斯模糊，保持云朵透明度固定
        # cloud_alpha = 1.0  # 保持云朵全不透明
        # cloud[:, :, 3] = cloud[:, :, 3] * cloud_alpha * 255
        
        # 将云朵与天空融合
        x_pos, y_pos = new_center[0] - cloud.shape[1] // 2, new_center[1] - cloud.shape[0] // 2
        x_min, x_max = max(0, x_pos), min(x_pos + cloud.shape[1], video_size[0])
        y_min, y_max = max(0, y_pos), min(y_pos + cloud.shape[0], video_size[1])
        cloud_x_min, cloud_x_max = max(0, -x_pos), min(cloud.shape[1], video_size[0] - x_pos)
        cloud_y_min, cloud_y_max = max(0, -y_pos), min(cloud.shape[0], video_size[1] - y_pos)
        
        alpha = cloud[cloud_y_min:cloud_y_max, cloud_x_min:cloud_x_max, 3][:, :, np.newaxis] / 255.
        
        sky[y_min:y_max, x_min:x_max, :3] = \
            sky[y_min:y_max, x_min:x_max, :3] * (1 - alpha) + \
            cloud[cloud_y_min:cloud_y_max, cloud_x_min:cloud_x_max, :3] * alpha
        
        # 更新云朵数据
        cloud_data['center'] = new_center
    
    out.write(sky)

out.release()
cv2.destroyAllWindows()
