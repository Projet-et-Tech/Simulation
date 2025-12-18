import time
import numpy as np
import sapien
import cv2
import threading
import queue

def normalize_image(img, min_val=0, max_val=255):
    """Flexible image normalization with custom range."""
    if img.size == 0:
        return np.zeros_like(img)
    
    img_min = img.min()
    img_max = img.max()
    
    if img_max > img_min:
        normalized_img = (
            (img - img_min) / (img_max - img_min) * 
            (max_val - min_val) + min_val
        )
        return normalized_img.astype(img.dtype)
    else:
        return np.full_like(img, min_val)

def display_images(processed_images):
    # Choose which image to display
    img_type = list(processed_images.keys())[0]
    img = processed_images[img_type]

    # Normalize and convert
    img = normalize_image(img).astype("uint8")
    img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGRA)
    
    cv2.imshow("Camera", img)
    key = cv2.waitKey(1)
    
    return key

class NoVirtualCamera:
    def __init__(self, *args, **kwargs):
        self.image_queue = queue.Queue(maxsize=1)

    def run(self):
        pass

class VirtualCamera:
    def __init__(self,
        scene,
        near=0.1,
        far=100,
        fovy=np.deg2rad(35),
        width=1920,
        height=1080,
        position=[-4, 0, 3],
        img_types=['Color', 'Depth', 'Segmentation']
        ):
        self.img_types = img_types
        # Compute the camera pose by specifying forward(x), left(y) and up(z)
        cam_pos = np.array(position)
        forward = -cam_pos / np.linalg.norm(cam_pos)
        left = np.cross([0, 0, 1], forward)
        left = left / np.linalg.norm(left)
        up = np.cross(forward, left)
        mat44 = np.eye(4)
        mat44[:3, :3] = np.stack([forward, left, up], axis=1)
        mat44[:3, 3] = cam_pos

        self.camera = scene.add_camera(
            name="camera",
            width=width,
            height=height,
            fovy=fovy,
            near=near,
            far=far,
        )
        self.camera.entity.set_pose(sapien.Pose(mat44))
        self.camera.take_picture()

    def optimized_image_capture(self):
        """Capture and process images from the camera."""
        processed_dict = {}
        for img_type in self.img_types:
            if img_type == 'Color':
                self.camera.take_picture()
                rgba_camera = self.camera.get_picture('Color')
                camera_color = np.clip(rgba_camera * 255, 0, 255).astype(np.uint8)
                processed_dict['Color'] = camera_color
            elif img_type == 'Depth':
                self.camera.take_picture()
                position = self.camera.get_picture('Position')
                depth = (-position[..., 2] * 1000.0).astype(np.uint16)
                processed_dict['Depth'] = depth
            elif img_type == 'Segmentation':
                self.camera.take_picture()
                seg_labels = self.camera.get_picture('Segmentation')
                segmentation = seg_labels[..., 0].astype(np.uint32)
                processed_dict['Segmentation'] = segmentation
            else:
                raise ValueError(f"Unsupported image type: {img_type}")
        
        return processed_dict

    def _image_capture_thread(self):
        """Thread function for image capture"""
        while True:
            try:
                t0 = time.time()
                processed_images = self.optimized_image_capture()
                
                if not self.image_queue.full():
                    self.image_queue.put((processed_images, time.time() - t0))
            
            except Exception as e:
                print(f"Capture thread error: {e}")
                break

    def run(self):
        self.image_queue = queue.Queue(maxsize=10)
        # Start image capture thread
        self.capture_thread = threading.Thread(
            target=self._image_capture_thread, 
            daemon=True  # Allows thread to exit when main program exits
        )
        self.capture_thread.start()
        