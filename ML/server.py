import os
import cv2
import numpy as np
import requests
import pickle
import base64
import time
import pandas as pd
from skimage.filters import gabor
from skimage.feature import canny
from scipy.stats import skew, kurtosis
from io import BytesIO
from PIL import Image

# Load Pre-trained Model
MODEL_PATH = r"final_gabor_best_model.pkl"
with open(MODEL_PATH, "rb") as model_file:
    model = pickle.load(model_file)

# Feature Extraction Functions
def apply_gabor_filters(image_gray, frequencies=[0.1, 0.3, 0.5], thetas=[0, np.pi/4, np.pi/2, 3*np.pi/4]):
    features = []
    for freq in frequencies:
        for theta in thetas:
            real, _ = gabor(image_gray, frequency=freq, theta=theta)
            features.append(np.mean(real))   # Mean response
            features.append(np.var(real))    # Variance response
            features.append(np.sum(real**2)) # Energy
    return features

def extract_canny_edges(image_gray):
    edges = canny(image_gray, sigma=1.0)
    return [np.sum(edges)]  # Sum of edges

def extract_histogram_features(image_gray, bins=10):
    hist, _ = np.histogram(image_gray, bins=bins, range=(0, 255))
    return hist.tolist()

def extract_statistical_features(image_gray):
    return [np.mean(image_gray), np.var(image_gray), skew(image_gray.flatten()), kurtosis(image_gray.flatten())]

def extract_features(image):
    """Extract features from an image."""
    try:
        gabor_features = apply_gabor_filters(image)
        edge_features = extract_canny_edges(image)
        hist_features = extract_histogram_features(image)
        stat_features = extract_statistical_features(image)
        return np.concatenate((gabor_features, edge_features, hist_features, stat_features)).reshape(1, -1)
    except Exception as e:
        print(f"Feature extraction error: {e}")
        return None

def decode_base64_image(encoded_string):
    """Decodes a base64-encoded image."""
    try:
        image_bytes = base64.b64decode(encoded_string)
        image = Image.open(BytesIO(image_bytes)).convert("L")  # Convert to grayscale
        return np.array(image)
    except Exception as e:
        print(f"Base64 decoding error: {e}")
        return None

def predict_terrain(image):
    """Extracts features and predicts the terrain type."""
    features = extract_features(image)
    if features is None:
        return None, None

    # Get class prediction and probability
    class_pred = model.predict(features)[0]
    prob = model.predict_proba(features).max()

    # Convert prediction to terrain type
    terrain_map = {0: "rocky_terrain", 1: "hard_terrain", 2: "sandy_terrain"}
    return terrain_map.get(class_pred, "unknown"), round(prob, 4)

def send_prediction(coordinates, terrain_class, accuracy):
    """Sends the predicted area to the server."""
    url = "http://localhost:5000/predicted_area"
    data = {
        "status": "success",
        "class": terrain_class,
        "accuracy": str(accuracy),
        "coordinate": coordinates
    }
    response = requests.post(url, json=data)
    if response.status_code == 200:
        print(f"Prediction sent: {data}")
    else:
        print(f"Error sending prediction: {response.text}")

def main():
    while True:
        try:
            # Check if the rover is near a flat area
            check_url = "http://localhost:5000/ml_check_area_temp"
            check_response = requests.get(check_url).json()

            if "coordinate" in check_response and "image" in check_response:
                coordinates = f"x={check_response['coordinate']['x']} y={check_response['coordinate']['y']} z={check_response['coordinate']['z']}"
                encoded_image = check_response["image"]

                print(f"Processing image at coordinates: {coordinates}")

                # Decode the base64 image
                image = decode_base64_image(encoded_image)
                if image is None:
                    continue

                # Predict terrain type
                terrain_class, accuracy = predict_terrain(image)
                if terrain_class is None:
                    continue

                print(f"Predicted Class: {terrain_class}, Accuracy: {accuracy}")

                # Send prediction
                send_prediction(coordinates, terrain_class, accuracy)

            else:
                print("No flat area detected, waiting...")
            
            time.sleep(5)  # Wait before the next request

        except Exception as e:
            print(f"Error in main loop: {e}")
            time.sleep(5)  # Retry after a short delay

if __name__ == "__main__":
    main()
