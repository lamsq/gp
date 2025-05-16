import face_recognition
import cv2
import numpy as np
from picamera2 import Picamera2
import time
import pickle

class FaceRecognizer:
    def __init__(self):
        # Load pre-trained face encodings
        print("[INFO] loading encodings...")
        with open("encodings.pickle", "rb") as f:
            data = pickle.loads(f.read())
        self.known_face_encodings = data["encodings"]
        self.known_face_names = data["names"]
        
        # Initialize camera
        self.picam2 = Picamera2()
        self.cv_scaler = 4  # this has to be a whole number
        self.frame_count = 0
        self.start_time = time.time()
        self.fps = 0
        
    def initialize_camera(self):
        """Initialize or reinitialize the camera"""
        try:
            self.picam2.stop()
        except:
            pass
        self.picam2.configure(self.picam2.create_preview_configuration(
            main={"format": 'XRGB8888', "size": (640, 480)}))
        self.picam2.start()
        
    def process_frame(self, frame):
        face_locations = []
        face_encodings = []
        face_names = []
        recognized = False
        
        # Resize the frame using cv_scaler to increase performance
        resized_frame = cv2.resize(frame, (0, 0), fx=(1/self.cv_scaler), fy=(1/self.cv_scaler))
        
        # Convert the image from BGR to RGB colour space
        rgb_resized_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)
        
        # Find all the faces and face encodings in the current frame of video
        face_locations = face_recognition.face_locations(rgb_resized_frame)
        face_encodings = face_recognition.face_encodings(rgb_resized_frame, face_locations, model='large')
        
        for face_encoding in face_encodings:
            # See if the face is a match for the known face(s)
            matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)
            name = "Unknown"
            
            # Use the known face with the smallest distance to the new face
            face_distances = face_recognition.face_distance(self.known_face_encodings, face_encoding)
            best_match_index = np.argmin(face_distances)
            
            if matches[best_match_index]:
                name = self.known_face_names[best_match_index]
                recognized = True
            face_names.append(name)
        
        return face_locations, face_names, recognized

    def draw_results(self, frame, face_locations, face_names):
        # Display the results
        for (top, right, bottom, left), name in zip(face_locations, face_names):
            # Scale back up face locations
            top *= self.cv_scaler
            right *= self.cv_scaler
            bottom *= self.cv_scaler
            left *= self.cv_scaler
            
            # Draw a box around the face
            cv2.rectangle(frame, (left, top), (right, bottom), (244, 42, 3), 3)
            
            # Draw a label with a name below the face
            cv2.rectangle(frame, (left -3, top - 35), (right+3, top), (244, 42, 3), cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(frame, name, (left + 6, top - 6), font, 1.0, (255, 255, 255), 1)
        
        return frame

    def calculate_fps(self):
        self.frame_count += 1
        elapsed_time = time.time() - self.start_time
        if elapsed_time > 1:
            self.fps = self.frame_count / elapsed_time
            self.frame_count = 0
            self.start_time = time.time()
        return self.fps

    def run_facial_recognition(self):
        try:
            print("started fr")
            self.initialize_camera()  # Reinitialize camera for each run
            
            # We'll process frames for a limited time (e.g., 10 seconds)
            end_time = time.time() + 20
            while time.time() < end_time:
                # Capture a frame from camera
                frame = self.picam2.capture_array()
                
                # Process the frame
                face_locations, face_names, recognized = self.process_frame(frame)
                
                # If recognized, return True immediately
                if recognized:
                    return True
                
                # Draw results
                display_frame = self.draw_results(frame, face_locations, face_names)
                
                # Calculate and update FPS
                current_fps = self.calculate_fps()
                
                # Attach FPS counter
                cv2.putText(display_frame, f"FPS: {current_fps:.1f}", (display_frame.shape[1] - 150, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # Display everything
                cv2.imshow('Video', display_frame)
                
                # Break the loop if 'q' is pressed
                if cv2.waitKey(1) == ord("q"):
                    break
            
            return False
        
        finally:
            # Clean up when done
            cv2.destroyAllWindows()
            self.picam2.stop()

# Singleton instance
face_recognizer = FaceRecognizer()

if __name__ == "__main__":
    # When run directly, show the video feed
    while True:
        frame = face_recognizer.picam2.capture_array()
        face_locations, face_names, _ = face_recognizer.process_frame(frame)
        display_frame = face_recognizer.draw_results(frame, face_locations, face_names)
        current_fps = face_recognizer.calculate_fps()
        cv2.putText(display_frame, f"FPS: {current_fps:.1f}", (display_frame.shape[1] - 150, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow('Video', display_frame)
        if cv2.waitKey(1) == ord("q"):
            break
    cv2.destroyAllWindows()
    face_recognizer.picam2.stop()