#!/usr/bin/env python3
"""Export a trained YOLO `best.pt` checkpoint to NCNN and/or LiteRT (TFLite).

Run this on a dev machine, not on the Raspberry Pi. Exporting needs the full
Ultralytics + PyTorch stack plus format-specific converters (PNNX for NCNN,
litert-torch/TensorFlow for LiteRT) -- see requirements-export.txt. The Pi
only needs the small runtime for whichever backend you deploy (see
base_detection/requirements.txt), never the export toolchain itself.

Why NCNN / TFLite instead of running best.pt directly: both are inference
runtimes built for ARM (NEON-accelerated), replacing the PyTorch backbone
compute with a much lighter, faster kernel -- see model_export/README.md
for the full comparison and quantization notes.

Usage:
    python export_model.py --weights ../models/best.pt --formats ncnn tflite
    python export_model.py --weights ../models/best.pt --formats tflite --tflite-quantize 8 --data coco8.yaml
"""

import argparse
import shutil
from pathlib import Path

from ultralytics import YOLO


def export_ncnn(weights: Path, imgsz: int, quantize: int, verify: bool) -> Path:
    model = YOLO(str(weights))
    out = Path(model.export(format="ncnn", imgsz=imgsz, quantize=quantize or None))

    # PNNX debug artifacts we don't need on the Pi.
    shutil.rmtree(out / "__pycache__", ignore_errors=True)

    if verify:
        verify_model(out, "ncnn")
    return out


def export_tflite(weights: Path, imgsz: int, quantize, data: str, verify: bool) -> Path:
    model = YOLO(str(weights))
    out = Path(model.export(format="tflite", imgsz=imgsz, quantize=quantize, data=data))

    if verify:
        verify_model(out, "tflite")
    return out


def verify_model(path: Path, label: str) -> None:
    """Smoke-test: load the exported model and run one prediction end to end."""
    import numpy as np

    dummy = (np.random.rand(480, 640, 3) * 255).astype("uint8")
    model = YOLO(str(path), task="detect")
    result = model.predict(dummy, verbose=False)[0]
    print(f"  [verify:{label}] loaded OK, class names={list(result.names.values())}")


def size_of(path: Path) -> float:
    if path.is_dir():
        return sum(f.stat().st_size for f in path.rglob("*") if f.is_file()) / 1e6
    return path.stat().st_size / 1e6


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--weights", default="../models/best.pt", help="Path to the trained best.pt checkpoint.")
    parser.add_argument("--formats", nargs="+", choices=["ncnn", "tflite"], default=["ncnn", "tflite"])
    parser.add_argument("--imgsz", type=int, default=640, help="Export input resolution (must match training imgsz).")
    parser.add_argument(
        "--ncnn-quantize",
        type=int,
        choices=[0, 16],
        default=16,
        help="0 = FP32, 16 = FP16 (default). NCNN has no Ultralytics-native INT8 export; "
        "see model_export/README.md for the separate ncnn2int8 toolchain.",
    )
    parser.add_argument(
        "--tflite-quantize",
        default="w8a32",
        help="32 = FP32, 'w8a32' = dynamic INT8 weights, no calibration data needed (default), "
        "8 = static INT8 (needs --data), 'w8a16' = INT8 weights + INT16 activations (needs --data).",
    )
    parser.add_argument(
        "--data",
        default=None,
        help="Calibration dataset YAML, required only for --tflite-quantize 8 or w8a16.",
    )
    parser.add_argument("--no-verify", action="store_true", help="Skip the post-export smoke prediction.")
    args = parser.parse_args()

    weights = Path(args.weights).resolve()
    if not weights.is_file():
        parser.error(f"weights not found: {weights}")

    tflite_quantize = args.tflite_quantize
    if str(tflite_quantize).lstrip("-").isdigit():
        tflite_quantize = int(tflite_quantize)
    if tflite_quantize in (8, "w8a16") and not args.data:
        parser.error("--tflite-quantize 8 / w8a16 requires --data <calibration.yaml>")

    verify = not args.no_verify
    outputs = []

    if "ncnn" in args.formats:
        print("Exporting NCNN...")
        outputs.append(("ncnn", export_ncnn(weights, args.imgsz, args.ncnn_quantize, verify)))

    if "tflite" in args.formats:
        print("Exporting TFLite (LiteRT)...")
        outputs.append(("tflite", export_tflite(weights, args.imgsz, tflite_quantize, args.data, verify)))

    print("\nDone:")
    for label, path in outputs:
        print(f"  {label:8s} {path}  ({size_of(path):.1f} MB)")
    print("\nPoint the detection scripts at one of the paths above with --model <path>.")


if __name__ == "__main__":
    main()
