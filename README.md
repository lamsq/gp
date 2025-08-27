#  Access Control with Facial Recognition

A real-time facial recognition security system built for the Raspberry Pi 4. It uses an infrared camera to identify users and controls a motor to grant access.

## Project Overview

I designed and engineered the complete hardware and software solution:

*   **Real-Time Recognition:** The system identifies faces in real-time using a custom-trained machine learning model.
*   **Hardware Integration:** It interfaces directly with hardware components via GPIO, including a camera, motor, and LED indicators.
*   **Custom Model:** The project includes scripts for capturing images and training a facial recognition model from scratch.
*   **Modular Architecture:** The code is structured for clarity and maintainability, with separate modules for image capture, recognition, and hardware control.

## Hardware Components

*   Raspberry Pi 4
*   Infrared Camera Module
*   Servo Motor
*   LED Display and Indicators

## Skills & Technologies Highlighted

This project demonstrates:

*   **Advanced Python Development:** Implemented the core application logic, data processing, and hardware control.
*   **Machine Learning:** Trained and deployed a custom facial recognition model using `dlib` and `face_recognition`.
*   **Computer Vision:** Utilized `OpenCV` for real-time image and video stream processing.
*   **IoT & Embedded Systems:** Engineered a complete IoT solution on a Raspberry Pi, controlling physical components with GPIO.
*   **System Architecture:** Designed a full-stack embedded system, from data collection and model training to final hardware actuation.

## Installation & Usage

### Prerequisites

*   Raspberry Pi 4
*   Python 3 and pip

### Setup

1.  **Clone the repository:**
    ```sh
    git clone git@github.com:lamsq/gp.git
    cd gp
    ```
2.  **Create and activate a virtual environment:**
    ```sh
    python3 -m venv face_rec
    source face_rec/bin/activate
    ```
3.  **Install dependencies:**
    ```sh
    pip install opencv-python dlib face-recognition imutils
    ```
4.  **Run the application:**
    ```sh
    python main.py
    ```
