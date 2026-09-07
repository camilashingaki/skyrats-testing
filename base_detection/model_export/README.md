# Exporting `best.pt` to NCNN / TFLite

`ultralytics.YOLO(...)` (used by both `Detector` classes in `../detect_base_*.py`)
auto-detects the model format from the path/extension. That means switching
runtime is just a matter of pointing `--model` at a different file -- no code
changes to the detection scripts are needed:

```bash
python detect_base_sky_mavlink.py --model models/best.pt                # PyTorch (default)
python detect_base_sky_mavlink.py --model models/best_ncnn_model        # NCNN
python detect_base_sky_mavlink.py --model models/best_w8a32.tflite      # TFLite (LiteRT)
```

## Why bother

Running `best.pt` directly means importing full PyTorch on the Pi and running
the backbone through it. NCNN and LiteRT are ARM-oriented inference runtimes:
they replace that backbone compute with NEON-accelerated kernels, which is
where the real FPS win on a Raspberry Pi comes from -- more so on a Pi 4
(Cortex-A72, no fp16 dot-product acceleration) than a Pi 5 (Cortex-A76).

| | Model size (this checkpoint, 2.5M params) | Runtime dependency on the Pi |
|---|---|---|
| `best.pt` (FP16) | 5.4 MB | full `torch` + `ultralytics` |
| `best_ncnn_model/` (FP16) | 4.7 MB | `ncnn` + `ultralytics` (still imports torch for pre/post-processing) |
| `best_w8a32.tflite` (dynamic INT8) | 2.7 MB | `ai-edge-litert` + `ultralytics` (still imports torch) |

Note both exported backends still go through `ultralytics.YOLO`, so `torch`
stays a dependency either way (it's used for the NMS/pre-post-processing
glue) -- the win here is inference speed on the backbone, not removing torch
from the Pi's RAM footprint. Dropping torch entirely would mean hand-writing
the pre/post-processing (letterbox resize, box decode, NMS) against the raw
`ncnn`/`ai-edge-litert` APIs instead of `ultralytics.YOLO` -- deliberately
not done here, since it's extra surface to get wrong for a gain that only
matters if RAM, not FPS, turns out to be the actual bottleneck on your Pi.

## Running the export (dev machine only)

```bash
pip install -r requirements-export.txt
cd model_export
python export_model.py --weights ../models/best.pt --formats ncnn tflite
```

This reproduces the two files already committed under `../models/`:
`best_ncnn_model/` (FP16) and `best_w8a32.tflite` (dynamic INT8, no
calibration data needed -- ~3.5x smaller than FP32 with no accuracy loss
from calibration, since only weights are quantized).

Re-run it whenever `best.pt` is retrained.

### Other quantization options

- `--tflite-quantize 32` -- plain FP32 TFLite, if you want a size/accuracy
  baseline to compare against.
- `--tflite-quantize 8 --data <dataset.yaml>` -- static INT8 (weights *and*
  activations), needs a calibration dataset (a handful of representative
  images referenced by a standard Ultralytics dataset YAML). Faster than
  `w8a32` but can lose a bit of accuracy; worth it only if `w8a32` isn't fast
  enough. Test it against your validation set before trusting it in flight.
- NCNN INT8 isn't exposed through this script: Ultralytics' NCNN export only
  supports FP32/FP16. True NCNN INT8 needs ncnn's own separate calibration
  toolchain (`ncnnoptimize` + `ncnn2table` + `ncnn2int8`); only worth setting
  up if `best_ncnn_model/` (FP16) turns out not to be fast enough.

## Benchmarking on the actual Pi

Numbers above are model/file size, not FPS -- that depends on the specific
Pi, thermal throttling, and camera resolution. Once deployed, measure with:

```bash
yolo benchmark model=models/best.pt imgsz=640 format=ncnn
yolo benchmark model=models/best.pt imgsz=640 format=tflite
```

`yolo benchmark` takes the source `best.pt` and re-exports it to the given
`format` itself (drop `format=` to benchmark every format at once), so it
needs the same heavy export-side dependencies as `export_model.py`. Run it on
the dev machine for a controlled comparison, or install
`requirements-export.txt` on the Pi temporarily if you specifically need
on-device numbers.
