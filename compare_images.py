import cv2
import os
import re
from skimage.metrics import structural_similarity as ssim

def load_image(path):
    img = cv2.imread(path)
    if img is not None:
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return None

def numerical_key(filename):
    # Extract numbers from the filename and convert to an integer tuple for proper comparison
    numbers = re.findall(r'\d+', filename)
    return tuple(int(num) for num in numbers) if numbers else (0,)

def main():
    images_dir = os.path.join(os.path.dirname(__file__), "images")
    image_files = sorted(
        [f for f in os.listdir(images_dir) if f.lower().endswith(('.png', '.jpg', '.jpeg'))],
        key=numerical_key
    )
    if len(image_files) < 4:
        print("Not enough images in directory.")
        return

    # Load first 3 images as templates
    templates = {}
    for template_file in image_files[:3]:
        template_path = os.path.join(images_dir, template_file)
        img = load_image(template_path)
        if img is not None:
            templates[template_file] = img

    # Compare each template with all other images to find the best match
    for ref_filename, ref_img in templates.items():
        best_ssim = -1
        best_match = None
        for candidate_file in image_files:
            if candidate_file == ref_filename:
                continue
            print(f"Comparing {ref_filename} with {candidate_file}")  # Debug: indicate progress
            candidate_path = os.path.join(images_dir, candidate_file)
            cand_img = load_image(candidate_path)
            if cand_img is None:
                continue
            # Resize candidate if sizes differ
            if cand_img.shape != ref_img.shape:
                cand_img = cv2.resize(cand_img, (ref_img.shape[1], ref_img.shape[0]))
            score = ssim(ref_img, cand_img)
            if score > best_ssim:
                best_ssim = score
                best_match = candidate_file
        if best_match:
            print(f"{ref_filename} matches closest with {best_match} (score: {best_ssim:.4f})")

if __name__ == "__main__":
    main()

