import os
import cv2
import numpy as np
import pandas as pd
from scipy.stats import skew, kurtosis
import joblib

class PreprocessImage:
    def __init__(self):
        pass  
    def prepare_image(self , image_path, target_size=(64, 64)):
        """Load, convert to grayscale, and resize image."""
        img = cv2.imread(image_path)
        if img is None:
            print(f"⚠️ Error: Unable to load image {image_path}")
            return None
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return cv2.resize(gray, target_size)


    def compute_fft(self ,image):
        """Compute FFT magnitude spectrum."""
        fft_image = cv2.dft(np.float32(image), flags=cv2.DFT_COMPLEX_OUTPUT)
        fft_shifted = np.fft.fftshift(fft_image)
        magnitude_spectrum = 20 * np.log(cv2.magnitude(fft_shifted[:, :, 0], fft_shifted[:, :, 1]) + 1)
        return magnitude_spectrum

    def ms_fdma_feature(self ,spectrum):
        return np.mean(spectrum)

    def ms_symmetry_feature(self ,spectrum):
        center = spectrum.shape[0] // 2
        upper, lower = spectrum[:center, :], np.flipud(spectrum[center:, :])
        sym_x = np.mean(np.abs(upper - lower))

        left, right = spectrum[:, :center], np.fliplr(spectrum[:, center:])
        sym_y = np.mean(np.abs(left - right))

        return sym_x, sym_y

    def ms_amplitude_moment(self ,spectrum):
        spectrum_flat = spectrum.flatten()
        return np.mean(spectrum_flat), np.var(spectrum_flat), skew(spectrum_flat), kurtosis(spectrum_flat)


    def extract_mgg_features(self ,image, num_scales=2, num_thresholds=2, base_dg=2):
        """Extracts 4 MGG features (2 scales × 2 thresholds)."""
        Pg_feature_vector = []
        h, w = image.shape

        for scale_index in range(num_scales):
            dg_scale = base_dg * (scale_index + 1)
            gu, gv = image[:, 1:] - image[:, :-1], image[1:, :] - image[:-1, :]
            gu, gv = np.pad(gu, [(0, 0), (0, 1)], mode='constant'), np.pad(gv, [(0, 1), (0, 0)], mode='constant')
            g = np.sqrt(gu**2 + gv**2)

            for j in range(1, num_thresholds + 1):
                thgj = j * dg_scale
                Pg_feature_vector.append(np.sum(g > thgj) / (h * w))

        return np.array(Pg_feature_vector)

    def extract_multiscale_features(self ,image, window_sizes=[5, 10]):
        """Extracts 2 MSGEF features using edge detection."""
        edges = cv2.Canny(cv2.equalizeHist(image), 10, 50)
        feature_vectors = []

        for window_size in window_sizes:
            step = window_size // 5
            features = []

            for u in range(0, image.shape[0] - window_size, step):
                for v in range(0, image.shape[1] - window_size, step):
                    window = edges[u:u+window_size, v:v+window_size]
                    edge_strength = np.sum(window) / (window_size * window_size)
                    features.append(edge_strength)

            feature_vectors.append(np.mean(features))

        return np.array(feature_vectors)
    
    

    def extract_features(self ,image_path):
        """Extract all features from an image."""
        grayscale_image = self.prepare_image(image_path)
        if grayscale_image is None:
            return None


        fft_magnitude = self.compute_fft(grayscale_image)
        fdma = self.ms_fdma_feature(fft_magnitude)
        sym_x, sym_y = self.ms_symmetry_feature(fft_magnitude)
        amp_mean, amp_variance, amp_skewness, amp_kurtosis = self.ms_amplitude_moment(fft_magnitude)
        msggf_features = self.extract_mgg_features(grayscale_image)  # Extract 4 features
        msgef_features = self.extract_multiscale_features(grayscale_image)  # Extract 2 features

        features = [fdma, sym_x, sym_y, amp_mean, amp_variance, amp_skewness, amp_kurtosis] + \
                list(msggf_features) + list(msgef_features)
        return np.array(features).reshape(1, -1)


def predict_image_class(model, image_path):
    """Predict the class of an image using the pre-trained model."""

    preprocess = PreprocessImage()
    features = preprocess.extract_features(image_path)
    if features is None:
        return None
    prediction = model.predict(features)
    class_mapping = {0: "hard", 1: "sandy", 2: "rocky"}
    return class_mapping.get(prediction[0], "Unknown")


if __name__ == '__main__':
    model_path = 'xgb_best_model.joblib'  # Path to your saved model
    image_path = 'Dataset/Rocky/0003ML0000001200100140E01_DRCL.JPG'  # Path to the image you want to predict


    model = joblib.load(model_path)


    predicted_class = predict_image_class(model, image_path)
    if predicted_class:
        print(f"The predicted class for the image is: {predicted_class}")
    else:
        print("Prediction could not be made due to an error in processing the image.")
