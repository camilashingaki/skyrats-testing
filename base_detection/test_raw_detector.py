#!/usr/bin/env python3
"""Run raw_detector.py (no torch) over every image in teste_imagens/.

Usage:
    python test_raw_detector.py --model models/best_ncnn_model
    python test_raw_detector.py --model models/best_w8a32.tflite --conf 0.4
"""

import argparse
from pathlib import Path

import cv2

from raw_detector import RawDetector

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default="models/best_ncnn_model", help="best_ncnn_model dir or a .tflite file.")
    parser.add_argument("--images-dir", default="teste_imagens", help="Folder of input images.")
    parser.add_argument("--out-dir", default=None, help="Defaults to <images-dir>/resultados.")
    parser.add_argument("--conf", type=float, default=0.5, help="Minimum confidence to count as a detection.")
    args = parser.parse_args()

    images_dir = Path(args.images_dir)
    out_dir = Path(args.out_dir) if args.out_dir else images_dir / "resultados"
    out_dir.mkdir(parents=True, exist_ok=True)

    images = sorted(p for p in images_dir.iterdir() if p.suffix.lower() in IMAGE_EXTS)
    if not images:
        print(f"No images found in {images_dir} (put .jpg/.png files there).")
        return

    detector = RawDetector(args.model, conf=args.conf, shape_only=True)
    print(f"Loaded {args.model}, classes considered: {[detector.class_names[i] for i in detector.class_indices]}")

    hits = 0
    for path in images:
        frame = cv2.imread(str(path))
        if frame is None:
            print(f"  {path.name}: could not read image, skipping")
            continue

        result = detector.detect(frame)
        annotated = detector.draw(frame, result)
        out_path = out_dir / f"{path.stem}_out.jpg"
        cv2.imwrite(str(out_path), annotated)

        if result is None:
            print(f"  {path.name}: no base found")
        else:
            hits += 1
            print(
                f"  {path.name}: {result['class_name']} conf={result['confidence']:.2f} "
                f"center=({result['center'][0]:.0f}, {result['center'][1]:.0f}) -> {out_path}"
            )

    print(f"\n{hits}/{len(images)} images with a detected base. Annotated images saved to {out_dir}/")


if __name__ == "__main__":
    main()
