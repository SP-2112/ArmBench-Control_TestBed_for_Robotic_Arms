"""
RLDS (Reinforcement Learning Dataset) creator for arm-bench
Records robot joint states, velocities, torques, and multi-camera video
"""
import os
import time
import json
import threading
from datetime import datetime
from typing import List, Dict, Optional
import numpy as np


class RLDSRecorder:
    """Records data in RLDS format for robot learning"""
    
    def __init__(self, output_dir: Optional[str] = None):
        self.output_dir = output_dir or os.path.expanduser("~/arm_bench_datasets")
        self.recording = False
        self.episode_count = 0
        self.current_episode = None
        
        # Recording threads
        self.record_thread = None
        self.camera_threads = []
        
        # Data buffers
        self.joint_data = []
        self.camera_data = {}
        
        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)
        
    def start_recording(self, episode_name: Optional[str] = None):
        """Start recording a new episode"""
        if self.recording:
            print("Already recording!")
            return False
        
        # Create episode directory
        if episode_name is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            episode_name = f"episode_{self.episode_count:04d}_{timestamp}"
        
        self.current_episode = episode_name
        episode_dir = os.path.join(self.output_dir, episode_name)
        os.makedirs(episode_dir, exist_ok=True)
        
        # Create subdirectories
        os.makedirs(os.path.join(episode_dir, "images"), exist_ok=True)
        os.makedirs(os.path.join(episode_dir, "joint_states"), exist_ok=True)
        
        print(f"Started recording episode: {episode_name}")
        print(f"Output directory: {episode_dir}")
        
        self.recording = True
        self.joint_data = []
        self.camera_data = {}
        
        # Start recording threads
        self.record_thread = threading.Thread(target=self._record_loop, daemon=True)
        self.record_thread.start()
        
        self.episode_count += 1
        return True
    
    def stop_recording(self):
        """Stop recording and save data"""
        if not self.recording:
            print("Not currently recording!")
            return False
        
        print("Stopping recording...")
        self.recording = False
        
        # Wait for threads to finish
        if self.record_thread:
            self.record_thread.join(timeout=5)
        
        # Save data
        self._save_episode_data()
        
        print(f"Episode saved: {self.current_episode}")
        return True
    
    def _record_loop(self):
        """Main recording loop"""
        start_time = time.time()
        frame_count = 0
        
        while self.recording:
            timestamp = time.time() - start_time
            
            # Record joint states
            joint_state = self._capture_joint_state(timestamp)
            self.joint_data.append(joint_state)
            
            # Record camera frames (would be done in separate threads)
            # self._capture_camera_frames(frame_count, timestamp)
            
            frame_count += 1
            time.sleep(0.01)  # 100Hz recording
    
    def _capture_joint_state(self, timestamp: float) -> Dict:
        """
        Capture current joint state
        
        Returns:
            Dictionary with joint positions, velocities, torques, temperatures
        """
        # In production, this would read from actual robot
        # For now, return simulated data
        
        num_joints = 6
        
        joint_state = {
            "timestamp": timestamp,
            "positions": [float(np.sin(timestamp + i)) for i in range(num_joints)],
            "velocities": [float(np.cos(timestamp + i)) for i in range(num_joints)],
            "efforts": [float(np.random.uniform(-1, 1)) for i in range(num_joints)],
            "temperatures": [float(30 + np.random.uniform(-2, 2)) for i in range(num_joints)],
            "joint_names": [f"joint_{i+1}" for i in range(num_joints)]
        }
        
        return joint_state
    
    def _capture_camera_frames(self, frame_count: int, timestamp: float):
        """Capture frames from all cameras"""
        # This would capture from actual cameras
        # Store in camera_data dictionary
        pass
    
    def _save_episode_data(self):
        """Save all recorded data to disk"""
        if not self.current_episode:
            return
        
        episode_dir = os.path.join(self.output_dir, self.current_episode)
        
        # Save joint states
        joint_states_file = os.path.join(episode_dir, "joint_states", "data.json")
        with open(joint_states_file, 'w') as f:
            json.dump(self.joint_data, f, indent=2)
        
        # Save metadata
        metadata = {
            "episode_name": self.current_episode,
            "num_frames": len(self.joint_data),
            "duration": self.joint_data[-1]["timestamp"] if self.joint_data else 0,
            "recorded_at": datetime.now().isoformat(),
            "cameras": list(self.camera_data.keys())
        }
        
        metadata_file = os.path.join(episode_dir, "metadata.json")
        with open(metadata_file, 'w') as f:
            json.dump(metadata, f, indent=2)
        
        print(f"Saved {len(self.joint_data)} frames")
    
    def add_camera(self, camera_id: int, camera_name: str):
        """Add a camera to record from"""
        self.camera_data[camera_name] = {
            "id": camera_id,
            "frames": []
        }
        print(f"Added camera: {camera_name} (ID: {camera_id})")
    
    def remove_camera(self, camera_name: str):
        """Remove a camera from recording"""
        if camera_name in self.camera_data:
            del self.camera_data[camera_name]
            print(f"Removed camera: {camera_name}")
    
    def list_episodes(self) -> List[str]:
        """List all recorded episodes"""
        if not os.path.exists(self.output_dir):
            return []
        
        episodes = [
            d for d in os.listdir(self.output_dir)
            if os.path.isdir(os.path.join(self.output_dir, d))
        ]
        
        return sorted(episodes)
    
    def convert_to_rlds(self, episode_name: str, output_format: str = "tfrecord"):
        """
        Convert episode to RLDS format
        
        Args:
            episode_name: Name of episode to convert
            output_format: 'tfrecord' or 'hdf5'
        """
        episode_dir = os.path.join(self.output_dir, episode_name)
        
        if not os.path.exists(episode_dir):
            print(f"Episode not found: {episode_name}")
            return False
        
        print(f"Converting {episode_name} to {output_format}...")
        
        # Load data
        joint_states_file = os.path.join(episode_dir, "joint_states", "data.json")
        
        if not os.path.exists(joint_states_file):
            print("Joint states file not found")
            return False
        
        with open(joint_states_file, 'r') as f:
            joint_data = json.load(f)
        
        # TODO: Implement actual RLDS conversion
        # This would create TFRecord or HDF5 files in the RLDS format
        # which includes:
        # - steps: list of (observation, action, reward, discount) tuples
        # - episode_metadata: metadata about the episode
        
        print("RLDS conversion not yet fully implemented")
        print("Joint data loaded successfully:")
        print(f"  - {len(joint_data)} timesteps")
        
        return True


class CameraManager:
    """Manages multiple cameras for recording"""
    
    def __init__(self):
        self.cameras = {}
        self.camera_names = {}
        
    def scan_cameras(self) -> List[Dict]:
        """Scan for available cameras"""
        cameras = []
        
        try:
            import cv2
            
            for idx in range(10):
                cap = cv2.VideoCapture(idx)
                if cap.isOpened():
                    cameras.append({
                        "id": idx,
                        "name": f"Camera {idx}",
                        "available": True
                    })
                    cap.release()
                    
        except ImportError:
            # Simulated cameras if OpenCV not available
            cameras = [
                {"id": 0, "name": "Camera 0", "available": True},
                {"id": 1, "name": "Camera 1", "available": True},
            ]
        
        self.cameras = {cam["id"]: cam for cam in cameras}
        return cameras
    
    def rename_camera(self, camera_id: int, new_name: str):
        """Rename a camera"""
        if camera_id in self.cameras:
            old_name = self.cameras[camera_id]["name"]
            self.cameras[camera_id]["name"] = new_name
            self.camera_names[camera_id] = new_name
            print(f"Renamed camera {camera_id}: {old_name} -> {new_name}")
        else:
            print(f"Camera {camera_id} not found")
    
    def get_camera_name(self, camera_id: int) -> Optional[str]:
        """Get camera name by ID"""
        if camera_id in self.camera_names:
            return self.camera_names[camera_id]
        elif camera_id in self.cameras:
            return self.cameras[camera_id]["name"]
        return None
    
    def start_preview(self, camera_id: int):
        """Start camera preview"""
        try:
            import cv2
            
            cap = cv2.VideoCapture(camera_id)
            if not cap.isOpened():
                print(f"Cannot open camera {camera_id}")
                return
            
            camera_name = self.get_camera_name(camera_id) or f"Camera {camera_id}"
            print(f"Starting preview for {camera_name}")
            print("Press 'q' to stop preview")
            
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                
                cv2.imshow(camera_name, frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            
            cap.release()
            cv2.destroyAllWindows()
            
        except ImportError:
            print("OpenCV not available. Cannot show preview.")
        except Exception as e:
            print(f"Error in camera preview: {e}")


def create_recorder(output_dir: Optional[str] = None) -> RLDSRecorder:
    """Create an RLDS recorder instance"""
    return RLDSRecorder(output_dir)


def create_camera_manager() -> CameraManager:
    """Create a camera manager instance"""
    return CameraManager()
