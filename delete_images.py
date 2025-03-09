import os

def identify_non_land_images_for_deletion(filtered_label_file_paths, calibrated_image_dir):
    """
    Identifies image files in 'calibrated_dir' that are NOT listed in the filtered label files (non-land images).

    Args:
        filtered_label_file_paths (list): List of paths to the filtered label files.
        calibrated_image_dir (str): Path to the 'calibrated' directory containing image files.

    Returns:
        list: List of full paths to image files that are identified as non-land images.
    """

    land_image_filenames = set()  # Use a set for faster lookups

    # 1. Collect filenames of 'land' images from the filtered label files
    for label_file_path in filtered_label_file_paths:
        if not os.path.exists(label_file_path) or os.stat(label_file_path).st_size == 0:
            print(f"Warning: Label file {label_file_path} is empty or missing! Skipping...")
            continue  # Skip empty or missing files

        with open(label_file_path, 'r') as infile:
            for line in infile:
                parts = line.strip().split()
                if parts:  # Avoid empty lines
                    image_filename_relative = os.path.basename(parts[0].strip())  # Extract filename only
                    land_image_filenames.add(image_filename_relative)  # Store only filenames

    # Debugging: Print collected land images
    print("\nLand image filenames from labels:", land_image_filenames)

    # 2. Get list of all image filenames in the 'calibrated' directory
    all_image_filenames_in_calibrated = set()
    for filename in os.listdir(calibrated_image_dir):  # Lists files directly in calibrated_dir
        if filename.lower().endswith(('.png', '.jpg', '.jpeg', '.tiff', '.bmp')):  # Check image extensions
            all_image_filenames_in_calibrated.add(filename)

    # Debugging: Print all images in calibrated directory
    print("\nAll images in calibrated directory:", all_image_filenames_in_calibrated)

    # 3. Identify non-land images (those in 'calibrated' but NOT in 'land_image_filenames')
    non_land_image_files_to_delete = []
    for image_filename_relative in all_image_filenames_in_calibrated:
        if image_filename_relative not in land_image_filenames:
            non_land_image_files_to_delete.append(os.path.join(calibrated_image_dir, image_filename_relative))

    # Debugging: Print identified non-land images
    print("\nIdentified non-land images:", non_land_image_files_to_delete)

    return non_land_image_files_to_delete


def main():
    """Main function to identify and (optionally) delete non-land images."""

    base_image_directory = r"C:\Users\Yuvraj\OneDrive\Files\Work\MARS\msl-images"  # **<-- VERIFY THIS PATH**
    calibrated_dir = os.path.join(base_image_directory, "calibrated")  
    filtered_labels_dir = os.path.join(base_image_directory, "filtered_labels")  

    filtered_label_files = [
        os.path.join(filtered_labels_dir, "train-land-images.txt"),
        os.path.join(filtered_labels_dir, "val-land-images.txt"),
        os.path.join(filtered_labels_dir, "test-land-images.txt")
    ]  

    # --- Identify non-land images ---
    non_land_image_files = identify_non_land_images_for_deletion(filtered_label_files, calibrated_dir)

    if not non_land_image_files:
        print("\nNo non-land image files identified for deletion.")
        print("This may indicate that all images are properly labeled or there is an issue with file paths.")
        return

    print("\n======================================================================")
    print("  WARNING:  FILES IDENTIFIED FOR DELETION!  ")
    print("======================================================================")
    print("The following image files have been identified as NON-LAND images:")
    print(" ")
    print("**PLEASE REVIEW THIS LIST CAREFULLY BEFORE PROCEEDING WITH DELETION.**")
    print("----------------------------------------------------------------------")
    for filepath in non_land_image_files:
        print(filepath)
    print("----------------------------------------------------------------------")
    print(f"\nTotal {len(non_land_image_files)} files identified for potential deletion.")

    confirmation = input("\n**ARE YOU SURE you want to DELETE these files PERMANENTLY? (yes/no):** ").strip().lower()

    if confirmation == 'yes':
        print("\n**DELETING FILES... PLEASE WAIT...**")
        deleted_count = 0
        for filepath in non_land_image_files:
            try:
                os.remove(filepath)
                deleted_count += 1
                print(f"Deleted: {filepath}")  # Optional: print each deleted file
            except OSError as e:
                print(f"Error deleting {filepath}: {e}")
        print(f"\n**DELETION PROCESS COMPLETE.**")
        print(f"Successfully deleted {deleted_count} files.")
        print(f"**{len(non_land_image_files) - deleted_count} files were NOT deleted due to errors.** (See error messages above)")
    else:
        print("\nDeletion process cancelled by user. No files were deleted.")
        print("**IMPORTANT:** No files have been deleted. This script only *identifies* files for potential deletion and requires explicit user confirmation to proceed with actual deletion.")


if __name__ == "__main__":
    main()
