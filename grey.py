import cv2
import numpy as np
import os

def prepare_image_for_mgg(image_path):
    """
    Loads an image from a file path, converts it to grayscale, and returns it as a NumPy array.

    Args:
        image_path (str): Path to the image file.

    Returns:
        numpy.ndarray or None: Grayscale image as a NumPy array, or None if image loading fails.
    """
    try:
        # 1. Load the image using OpenCV
        img_bgr = cv2.imread(image_path)

        # Check if image loading was successful
        if img_bgr is None:
            print(f"Error: Could not load image from path: {image_path}")
            return None

        # 2. Convert the image to grayscale
        grayscale_img = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        
        return grayscale_img

    except Exception as e:
        print(f"An error occurred while processing image: {image_path}")
        print(f"Error details: {e}")
        return None
    
if __name__ == '__main__':
    from msgggf import extract_mgg_features_standalone 
    from msgef import extract_multiscale_features
    
    image_file_path = r"msl-images\calibrated\0066MR0002930160103294E01_DRCL.JPG"   
   
    if not os.path.exists(image_file_path):
        dummy_image = np.zeros((100, 100, 3), dtype=np.uint8) 
        cv2.imwrite(image_file_path, dummy_image)
        print(f"Created a dummy image '{image_file_path}'. Replace it with your actual image.")


    grayscale_image = prepare_image_for_mgg(image_file_path)

    if grayscale_image is not None:
        print("\nGrayscale Image loaded successfully!")
        print("Grayscale image shape:", grayscale_image.shape) 
        cv2.imshow('image', grayscale_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

       
        mgg_features = extract_mgg_features_standalone(grayscale_image, num_scales=2, num_thresholds=5, base_dg=5)
        mse_features = extract_multiscale_features(grayscale_image, window_sizes=[5, 10])

        print("\nExtracted MGG Features from the image:")
        print(mgg_features)
        print(mse_features)
        

    else:
        print("\nImage loading or processing failed.")
