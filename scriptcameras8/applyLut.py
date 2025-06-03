import cv2
import numpy as np

# Charger la LUT
def load_lut(file_path, size=17):
    with open(file_path, 'r') as f:
        lines = f.readlines()
    lut = []
    for line in lines:
        if line.startswith('#') or line.startswith('TITLE') or line.startswith('DOMAIN') or line.startswith('LUT_3D_SIZE'):
            continue
        parts = line.strip().split()
        if len(parts) == 3:
            lut.append([float(p) for p in parts])
    lut = np.array(lut).reshape((size, size, size, 3))
    return lut

# Appliquer la LUT
def apply_lut(image, lut):
    img = image.astype(np.float32) / 255.0
    img = np.clip(img, 0, 1)

    idx = (img * (lut.shape[0] - 1)).astype(int)
    result = lut[idx[:,:,0], idx[:,:,1], idx[:,:,2]]
    result = np.clip(result * 255, 0, 255).astype(np.uint8)
    return result

# Exemple d'utilisation
lut = load_lut('kodachrome40_perime.cube')

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.resize(frame, (640, 480))
    frame_lut = apply_lut(frame, lut)

    cv2.imshow('Kodachrome 40', frame_lut)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

